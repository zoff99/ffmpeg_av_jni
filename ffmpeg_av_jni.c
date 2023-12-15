#define _GNU_SOURCE

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <dirent.h>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>

#include <fcntl.h>
#include <errno.h>

#include <pthread.h>

#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavfilter/avfilter.h>
#include "libavfilter/buffersink.h"
#include "libavfilter/buffersrc.h"
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/time.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>

#ifdef __APPLE__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifndef OS_WIN32
#include <sys/time.h>
#endif

#ifdef __linux__
#include <X11/Xlib.h>
#endif

#include <jni.h>

// ----------- version -----------
// ----------- version -----------
#define VERSION_MAJOR 0
#define VERSION_MINOR 99
#define VERSION_PATCH 10
static const char global_version_string[] = "0.99.10";
// ----------- version -----------
// ----------- version -----------

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define c_sleep(x) usleep(1000*x)

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


#define CURRENT_LOG_LEVEL 9 // 0 -> error, 1 -> warn, 2 -> info, 9 -> debug
#define MAX_LOG_LINE_LENGTH 1000
#define MAX_FULL_PATH_LENGTH 1000

#ifdef __cplusplus
extern "C" {
#endif

// ############## FIFO ##############
// ############## FIFO ##############

typedef struct {
    void* data;
    size_t size;
    size_t head;
    size_t tail;
} fifo_buffer_t;

fifo_buffer_t* fifo_buffer_create(size_t size) {
    fifo_buffer_t* buffer = calloc(1, sizeof(fifo_buffer_t));
    buffer->data = calloc(1, size);
    buffer->size = size;
    buffer->head = 0;
    buffer->tail = 0;
    return buffer;
}

void fifo_buffer_destroy(fifo_buffer_t* buffer) {
    free(buffer->data);
    free(buffer);
}

size_t fifo_buffer_free(fifo_buffer_t* buffer) {
    if ((buffer->tail == buffer->head) && (buffer->head == 0)){
        return (buffer->size - 1);
    } else {
        if (buffer->tail > 0)
        {
            if (buffer->tail < buffer->head)
            {
                // move data to beginning of the buffer
                memmove(buffer->data, buffer->data + buffer->tail, buffer->head - buffer->tail);
            }
            buffer->head -= buffer->tail;
            buffer->tail = (size_t)0;
        }
        return buffer->size - buffer->head - 1;
    }
}

size_t fifo_buffer_data_available(fifo_buffer_t* buffer) {
    if ((buffer->tail == buffer->head) && (buffer->head == (size_t)0)) {
        return (size_t)0;
    } else {
        return buffer->head - buffer->tail;
    }
}

size_t fifo_buffer_write(fifo_buffer_t* buffer, const void* data, size_t size) {
    size_t available = fifo_buffer_free(buffer);
    if (size > available) {
        size = available;
    }
    memcpy(buffer->data + buffer->head, data, size);
    buffer->head += size;
    return size;
}

size_t fifo_buffer_read(fifo_buffer_t* buffer, void* data, size_t size) {
    size_t available = fifo_buffer_data_available(buffer);
    if (size > available) {
        size = available;
    }
    memcpy(data, buffer->data + buffer->tail, size);
    buffer->tail += size;
    return size;
}

// ############## FIFO ##############
// ############## FIFO ##############

// ----- JNI stuff -----
JNIEnv *jnienv;
JavaVM *cachedJVM = NULL;


// gives a counter value that increaes every millisecond
static uint64_t ffmpegav_current_time_monotonic_default()
{
    uint64_t time = 0;
#ifdef OS_WIN32
    /* Must hold mono_time->last_clock_lock here */

    /* GetTickCount provides only a 32 bit counter, but we can't use
     * GetTickCount64 for backwards compatibility, so we handle wraparound
     * ourselves.
     */
    uint32_t ticks = GetTickCount();

    /* the higher 32 bits count the number of wrap arounds */
    uint64_t old_ovf = mono_time->time & ~((uint64_t)UINT32_MAX);

    /* Check if time has decreased because of 32 bit wrap from GetTickCount() */
    if (ticks < mono_time->last_clock_mono) {
        /* account for overflow */
        old_ovf += UINT32_MAX + UINT64_C(1);
    }

    if (mono_time->last_clock_update) {
        mono_time->last_clock_mono = ticks;
        mono_time->last_clock_update = false;
    }

    /* splice the low and high bits back together */
    time = old_ovf + ticks;
#else
    struct timespec clock_mono;
#if defined(__APPLE__)
    clock_serv_t muhclock;
    mach_timespec_t machtime;

    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &muhclock);
    clock_get_time(muhclock, &machtime);
    mach_port_deallocate(mach_task_self(), muhclock);

    clock_mono.tv_sec = machtime.tv_sec;
    clock_mono.tv_nsec = machtime.tv_nsec;
#else
    clock_gettime(CLOCK_MONOTONIC, &clock_mono);
#endif
    time = 1000ULL * clock_mono.tv_sec + (clock_mono.tv_nsec / 1000000ULL);
#endif
    return time;
}

static size_t ffmpegav_xnet_pack_u16(uint8_t *bytes, uint16_t v)
{
    bytes[0] = (v >> 8) & 0xff;
    bytes[1] = v & 0xff;
    return sizeof(v);
}

static size_t ffmpegav_xnet_pack_u32(uint8_t *bytes, uint32_t v)
{
    uint8_t *p = bytes;
    p += ffmpegav_xnet_pack_u16(p, (v >> 16) & 0xffff);
    p += ffmpegav_xnet_pack_u16(p, v & 0xffff);
    return p - bytes;
}

static size_t ffmpegav_xnet_unpack_u16(const uint8_t *bytes, uint16_t *v)
{
    uint8_t hi = bytes[0];
    uint8_t lo = bytes[1];
    *v = ((uint16_t)hi << 8) | lo;
    return sizeof(*v);
}

static size_t ffmpegav_xnet_unpack_u32(const uint8_t *bytes, uint32_t *v)
{
    const uint8_t *p = bytes;
    uint16_t hi;
    uint16_t lo;
    p += ffmpegav_xnet_unpack_u16(p, &hi);
    p += ffmpegav_xnet_unpack_u16(p, &lo);
    *v = ((uint32_t)hi << 16) | lo;
    return p - bytes;
}

// ------------- JNI -------------
// ------------- JNI -------------
// ------------- JNI -------------

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *jvm, void *reserved)
{
    JNIEnv *env_this;
    cachedJVM = jvm;
    if((*jvm)->GetEnv(jvm, (void **) &env_this, JNI_VERSION_1_6))
    {
        return JNI_ERR;
    }
    return JNI_VERSION_1_6;
}

JNIEnv *jni_getenv()
{
    JNIEnv *env_this;
    (*cachedJVM)->GetEnv(cachedJVM, (void **) &env_this, JNI_VERSION_1_6);
    return env_this;
}

int java_find_class_global(char *name, jclass *ret)
{
    JNIEnv *jnienv2;
    jnienv2 = jni_getenv();
    *ret = (*jnienv2)->FindClass(jnienv2, name);
    if(!*ret)
    {
        return 0;
    }
    *ret = (*jnienv2)->NewGlobalRef(jnienv2, *ret);
    return 1;
}


jclass AVActivity = NULL;
jmethodID callback_video_capture_frame_pts_cb_method = NULL;
jmethodID callback_video_capture_frame_too_small_cb_method = NULL;
jmethodID callback_audio_capture_frame_pts_cb_method = NULL;
jmethodID callback_audio_capture_frame_too_small_cb_method = NULL;

// --------- AV VARS ---------
// --------- AV VARS ---------
// --------- AV VARS ---------
int global_audio_delay_factor = 0;

AVInputFormat *inputFormat_audio = NULL;
AVFormatContext *formatContext_audio = NULL;
AVCodecContext *global_audio_codec_ctx = NULL;
AVDictionary *options_audio = NULL;
int audio_stream_index = -1;
AVCodec *audio_codec = NULL;
bool global_audio_in_capture_running = false;
static pthread_t ffmpeg_thread_audio_in_capture;
AVRational time_base_audio = (AVRational) {0, 0};
#define DEFAULT_SCREEN_CAPTURE_PULSE_DEVICE "default"
int apply_audio_filter = 0;
char *resources_dir_path = NULL;
const int resources_dir_path_maxlen = 1000;

AVInputFormat *inputFormat_video = NULL;
AVFormatContext *formatContext_video = NULL;
AVCodecContext *global_video_codec_ctx = NULL;
AVDictionary *options_video = NULL;
int video_stream_index = -1;
AVCodec *video_codec = NULL;
bool global_video_in_capture_running = false;
static pthread_t ffmpeg_thread_video_in_capture;
AVRational time_base_video = (AVRational) {0, 0};

pthread_mutex_t vsend___mutex;
pthread_mutex_t vscale___mutex;
pthread_mutex_t vbuffer2___mutex;
#define VIDEO_SEND_THREAD_COUNT_MAX 10
int vsend_thread_count = 0;

int sws_scale_algo = SWS_POINT; // SWS_POINT SWS_FAST_BILINEAR SWS_BILINEAR SWS_BICUBIC SWS_SINC SWS_LANCZOS
int output_width = 640;
int output_height = 480;

const int audio_frame_size_ms = 60;
const int out_channels = 1; // keep in sync with `out_channel_layout`
const uint64_t out_channel_layout = AV_CH_LAYOUT_MONO; // AV_CH_LAYOUT_MONO or AV_CH_LAYOUT_STEREO;
const int out_bytes_per_sample = 2; // 2 byte per PCM16 sample
const int out_samples = audio_frame_size_ms * 48; // X ms @ 48000Hz
const int out_sample_rate = 48000; // fixed at 48000Hz
const int temp_audio_buf_sizes = 600000; // fixed buffer


uint8_t *video_buffer_1 = NULL;
uint8_t *video_buffer_1_u = NULL;
uint8_t *video_buffer_1_v = NULL;
long video_buffer_1_size = 0;
int video_buffer_1_width = 0;
int video_buffer_1_height = 0;
int video_buffer_1_y_size = 0;
int video_buffer_1_u_size = 0;
int video_buffer_1_v_size = 0;

uint8_t *video_buffer_2 = NULL;
uint8_t *video_buffer_2_u = NULL;
uint8_t *video_buffer_2_v = NULL;
long video_buffer_2_size = 0;
int video_buffer_2_y_size = 0;
int video_buffer_2_u_size = 0;
int video_buffer_2_v_size = 0;

uint8_t *audio_buffer_pcm_1 = NULL;
long audio_buffer_pcm_1_size = 0;

uint8_t *audio_buffer_pcm_2 = NULL;
long audio_buffer_pcm_2_size = 0;
// --------- AV VARS ---------
// --------- AV VARS ---------
// --------- AV VARS ---------


static void reset_video_in_values()
{
    inputFormat_video = NULL;
    options_video = NULL;
    video_stream_index = -1;
}

static void reset_audio_in_values()
{
    inputFormat_audio = NULL;
    options_audio = NULL;
    audio_stream_index = -1;
}

/**
 * @brief Delays the execution of the current thread for a specified number of milliseconds.
 *
 * @param ms The number of milliseconds to delay the execution of the current thread.
 */
static void yieldcpu(uint32_t ms)
{
    usleep(1000 * ms);
}

struct vsend_data {
    int output_width;
    int output_height;
    struct SwsContext *scaler_ctx;
    AVFrame* frame2;
    int source_format;
    uint64_t pin_ts_ms;
    int estimated_fps;
    uint8_t *dst_yuv_buffer[3];
};

static void *thread_v_send_bg_func(void *data)
{
    struct vsend_data *vs = (struct vsend_data *) data;
    int output_width = vs->output_width;
    int output_height = vs->output_height;
    struct SwsContext *scaler_ctx = vs->scaler_ctx;
    AVFrame* frame = vs->frame2;
    uint64_t pint_ts_ms = vs->pin_ts_ms;
    int source_format = vs->source_format;
    int estimated_fps = vs->estimated_fps;

    // Convert the video frame to YUV
    int planes_stride[3];
    planes_stride[0] = av_image_get_linesize(AV_PIX_FMT_YUV420P, output_width, 0);
    planes_stride[1] = av_image_get_linesize(AV_PIX_FMT_YUV420P, output_width, 1);
    planes_stride[2] = av_image_get_linesize(AV_PIX_FMT_YUV420P, output_width, 2);
    // fprintf(stderr, "VideoFrame:strides:%d %d %d\n",planes_stride[0],planes_stride[1],planes_stride[2]);

    uint8_t *yuv_buffer = (uint8_t *)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_YUV420P,
            output_width, output_height, 1));
    if (yuv_buffer == NULL) {
        fprintf(stderr, "Error: could not allocate YUV buffer\n");

        av_frame_unref(frame);
        free(vs);
        pthread_mutex_lock(&vsend___mutex);
        vsend_thread_count--;
        if (vsend_thread_count < 0)
        {
            vsend_thread_count = 0;
        }
        pthread_mutex_unlock(&vsend___mutex);

        return NULL;
    }

    uint8_t *dst_yuv_buffer[3];
    dst_yuv_buffer[0] = yuv_buffer;
    dst_yuv_buffer[1] = yuv_buffer + (output_width * output_height);
    dst_yuv_buffer[2] = dst_yuv_buffer[1] + ((output_width * output_height) / 4);

    pthread_mutex_lock(&vscale___mutex);
    sws_scale(scaler_ctx, (const uint8_t * const*)frame->data, frame->linesize, 0, global_video_codec_ctx->height,
            dst_yuv_buffer, planes_stride);
    pthread_mutex_unlock(&vscale___mutex);

    JNIEnv *jnienv2 = NULL;
    if (jnienv2 == NULL)
    {
        JavaVMAttachArgs args;
        args.version = JNI_VERSION_1_6; // choose your JNI version
        args.name = "t_ff_jvcb2"; // you might want to give the java thread a name
        args.group = NULL; // you might want to assign the java thread to a ThreadGroup
        if (cachedJVM)
        {
            (*cachedJVM)->AttachCurrentThread(cachedJVM, (void **) &jnienv2, &args);
        }
    }

    pthread_mutex_lock(&vbuffer2___mutex);
    if (
        (video_buffer_2_y_size >= (planes_stride[0] * output_height)) &&
        (video_buffer_2_u_size >= (planes_stride[1] * (output_height / 2))) &&
        (video_buffer_2_v_size >= (planes_stride[2] * (output_height / 2)))
        )
    {
        memcpy(video_buffer_2, dst_yuv_buffer[0], planes_stride[0] * output_height);
        memcpy(video_buffer_2_u, dst_yuv_buffer[1], planes_stride[1] * (output_height / 2));
        memcpy(video_buffer_2_v, dst_yuv_buffer[2], planes_stride[2] * (output_height / 2));
        pthread_mutex_unlock(&vbuffer2___mutex);

        if (jnienv2 != NULL) {
        uint64_t video_frame_age_ms = ffmpegav_current_time_monotonic_default() - pint_ts_ms;
        if ((video_frame_age_ms < 1) || (video_frame_age_ms > 300))
        {
            video_frame_age_ms = 0;
        }
        (*jnienv2)->CallStaticVoidMethod(jnienv2, AVActivity,
                callback_video_capture_frame_pts_cb_method,
                (jlong)output_width,
                (jlong)output_height,
                (jlong)global_video_codec_ctx->width,
                (jlong)global_video_codec_ctx->height,
                (jlong)video_frame_age_ms,
                (jint)estimated_fps,
                (jint)source_format
            );
        }
        else
        {
            fprintf(stderr, "could not attach thread to JVM [1]\n");
        }
    }
    else
    {
        pthread_mutex_unlock(&vbuffer2___mutex);
        fprintf(stderr, "video buffer too small for video frame data\n");
        if (jnienv2 != NULL) {
        (*jnienv2)->CallStaticVoidMethod(jnienv2, AVActivity,
                callback_video_capture_frame_too_small_cb_method,
                (jint)(planes_stride[0] * output_height),
                (jint)(planes_stride[1] * (output_height / 2)),
                (jint)(planes_stride[2] * (output_height / 2))
            );
        }
        else
        {
            fprintf(stderr, "could not attach thread to JVM [2]\n");
        }
    }

    av_free(yuv_buffer);
    av_frame_unref(frame);
    free(vs);

    if (cachedJVM)
    {
        (*cachedJVM)->DetachCurrentThread(cachedJVM);
    }

    pthread_mutex_lock(&vsend___mutex);
    vsend_thread_count--;
    if (vsend_thread_count < 0)
    {
        vsend_thread_count = 0;
    }
    pthread_mutex_unlock(&vsend___mutex);
    return NULL;

}

static void *ffmpeg_thread_video_in_capture_func(void *data)
{
    AVPacket packet;
    AVFrame *frame = NULL;
    int ret = 0;
    int estimated_fps = 0;
    uint64_t fps_measure_timestamp = 0;
    int32_t count_video_frames = 0;
    int source_format = -1;

    if (global_video_codec_ctx == NULL) {
        fprintf(stderr, "video codec is NULL\n");
        return NULL;
    }
    if (inputFormat_video == NULL) {
        fprintf(stderr, "inputFormat_video is NULL\n");
        return NULL;
    }
    if (video_stream_index == -1) {
        fprintf(stderr, "video_stream_index is -1\n");
        return NULL;
    }

    // Allocate a frame for decoding
    frame = av_frame_alloc();
    if (!frame) {
        fprintf(stderr, "Could not allocate frame\n");
        printf("ffmpeg Video Capture Thread:ERROR:001:thread exit!\n");
        return NULL;
    }


    // Allocate a buffer for the YUV data
    int yuv_buffer_bytes_size = av_image_get_buffer_size(AV_PIX_FMT_YUV420P, output_width, output_height, 1);
    uint8_t *yuv_buffer = (uint8_t *)av_malloc(yuv_buffer_bytes_size);
    if (yuv_buffer == NULL) {
        fprintf(stderr, "Error: could not allocate YUV buffer\n");
        printf("ffmpeg Video Capture Thread:ERROR:002:thread exit!\n");
        av_frame_free(&frame);
        return NULL;
    }

/*
    uint8_t *dst_yuv_buffer[3];
    dst_yuv_buffer[0] = yuv_buffer;
    dst_yuv_buffer[1] = yuv_buffer + (output_width * output_height);
    dst_yuv_buffer[2] = dst_yuv_buffer[1] + ((output_width * output_height) / 4);
*/

    if (global_video_codec_ctx->pix_fmt < 0 || global_video_codec_ctx->pix_fmt >= AV_PIX_FMT_NB)
    {
        printf("pxfmt BAD!!!!!! fmt:%d maxfmt: %d\n", global_video_codec_ctx->pix_fmt, AV_PIX_FMT_NB);
    }
    else
    {
        AVPixFmtDescriptor pixfmt_desc = *av_pix_fmt_desc_get(global_video_codec_ctx->pix_fmt);
        printf("pxfmt OK fmt: %d fmtname: %s\n", global_video_codec_ctx->pix_fmt, pixfmt_desc.name);
    }

    // Create a scaler context to convert the video to YUV
    struct SwsContext *scaler_ctx = sws_getContext(
            global_video_codec_ctx->width,
            global_video_codec_ctx->height,
            global_video_codec_ctx->pix_fmt, output_width, output_height,
            AV_PIX_FMT_YUV420P, sws_scale_algo, NULL, NULL, NULL);
    if (scaler_ctx == NULL) {
        fprintf(stderr, "Error: could not create scaler context\n");
        printf("ffmpeg Video Capture Thread:ERROR:003:thread exit!\n");
        av_frame_free(&frame);
        av_free(yuv_buffer);
        return NULL;
    }

    fprintf(stderr, "SwsContext: %dx%d -> %dx%d\n",
        global_video_codec_ctx->width, global_video_codec_ctx->height, output_width, output_height);

    JNIEnv *jnienv2 = NULL;
    if (jnienv2 == NULL)
    {
        JavaVMAttachArgs args;
        args.version = JNI_VERSION_1_6; // choose your JNI version
        args.name = "t_ff_jvcb"; // you might want to give the java thread a name
        args.group = NULL; // you might want to assign the java thread to a ThreadGroup
        if (cachedJVM)
        {
            (*cachedJVM)->AttachCurrentThread(cachedJVM, (void **) &jnienv2, &args);
        }
    }

    source_format = global_video_codec_ctx->codec_id;
    fps_measure_timestamp = ffmpegav_current_time_monotonic_default();
    while (global_video_in_capture_running == true)
    {
        if (av_read_frame(formatContext_video, &packet) >= 0)
        {
            if (packet.stream_index == video_stream_index)
            {
                // Decode video packet
                ret = avcodec_send_packet(global_video_codec_ctx, &packet);
                if (ret < 0)
                {
                    fprintf(stderr, "Error sending video packet for decoding\n");
                    break;
                }
                while (ret >= 0)
                {
                    ret = avcodec_receive_frame(global_video_codec_ctx, frame);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    {
                        break;
                    }
                    else if (ret < 0)
                    {
                        fprintf(stderr, "Error during video decoding\n");
                        break;
                    }

                    // calculate fps on every 5th video frame ---------------------------------
                    count_video_frames++;
                    if (count_video_frames > 4)
                    {
                        const uint64_t fps_measure_timestamp_end = ffmpegav_current_time_monotonic_default();
                        const float delta_ms = (float)(fps_measure_timestamp_end - fps_measure_timestamp);
                        if ((delta_ms > 1) && (delta_ms < 8000)) {
                            estimated_fps = (int)(1000.0f / (delta_ms / (float)count_video_frames));
                        } else {
                            estimated_fps = 0; // set fps to zero on any errors or issues
                        }
                        fps_measure_timestamp = fps_measure_timestamp_end;
                        count_video_frames = 0;
                    }
                    // calculate fps on every 5th video frame ---------------------------------

                    // --------------------
                    // --------------------
                    // ---- Background ----
                    // --------------------
                    // --------------------
                    AVFrame* frame2 = av_frame_clone(frame);
                    av_frame_unref(frame);

                    struct vsend_data *vs = malloc(sizeof(struct vsend_data));
                    vs->output_width = output_width;
                    vs->output_height = output_height;
                    vs->scaler_ctx = scaler_ctx;
                    vs->frame2 = frame2;
                    vs->source_format = source_format;
                    vs->pin_ts_ms = ffmpegav_current_time_monotonic_default();
                    vs->estimated_fps = estimated_fps;

                    bool vsend_thread_spawn = 1;
                    pthread_t thread_v_send_bg;
                    pthread_mutex_lock(&vsend___mutex);
                    if (vsend_thread_count > VIDEO_SEND_THREAD_COUNT_MAX)
                    {
                        vsend_thread_spawn = 0;
                    }
                    pthread_mutex_unlock(&vsend___mutex);
                    if (vsend_thread_spawn == 1)
                    {
                        if (pthread_create(&thread_v_send_bg, NULL, thread_v_send_bg_func, (void *)vs) != 0)
                        {
                            printf("VSend Thread create failed\n");
                            av_frame_unref(frame2);
                            free(vs);
                        }
                        else
                        {
                            pthread_mutex_lock(&vsend___mutex);
                            vsend_thread_count++;
                            // printf("vsend_thread_count:%d\n", vsend_thread_count);
                            pthread_mutex_unlock(&vsend___mutex);
#ifdef __linux__
                            pthread_setname_np(thread_v_send_bg, "t_vsend");
#endif
                            if (pthread_detach(thread_v_send_bg))
                            {
                                printf("error detaching VSend Thread\n");
                            }
                        }
                    }
                    else
                    {
                        av_frame_unref(frame2);
                        free(vs);
                    }

                    // --------------------
                    // --------------------
                    // ---- Background ----
                    // --------------------
                    // --------------------
                }
            }
            av_packet_unref(&packet);
        }
        // yieldcpu(100);
    }

    // HINT: wait for all video bg send threads to finish ------------
    long loop_count = 0;
    const max_loop_count = 50;
    while (true)
    {
        pthread_mutex_lock(&vsend___mutex);
        if (vsend_thread_count < 1)
        {
            pthread_mutex_unlock(&vsend___mutex);
            break;
        }
        pthread_mutex_unlock(&vsend___mutex);
        yieldcpu(2);
        loop_count++;
        if (loop_count > max_loop_count)
        {
            fprintf(stderr, "break waiting for video capture threads to finish\n");
            break;
        }
    }
    // HINT: wait for all video bg send threads to finish ------------

    if (cachedJVM)
    {
        (*cachedJVM)->DetachCurrentThread(cachedJVM);
    }

    av_frame_free(&frame);
    av_free(yuv_buffer);
    sws_freeContext(scaler_ctx);

    printf("ffmpeg Video Capture Thread:Clean thread exit!\n");
    return NULL;
}

static void *ffmpeg_thread_audio_in_capture_func(void *data)
{
    AVPacket packet;
    AVFrame *frame = NULL;
    AVFrame *oframe = NULL;
    int ret = 0;
    int num_samples = 0;
    int audio_delay_in_bytes = 0;
    uint8_t **converted_samples = NULL;
    int use_filter = 0;
    int filter_init_error = 0;

    if (global_audio_codec_ctx == NULL) {
        fprintf(stderr, "audio codec is NULL\n");
        return NULL;
    }
    if (inputFormat_audio == NULL) {
        fprintf(stderr, "inputFormat_audio is NULL\n");
        return NULL;
    }
    if (audio_stream_index == -1) {
        fprintf(stderr, "audio_stream_index is -1\n");
        return NULL;
    }

    // Allocate a frame for decoding
    frame = av_frame_alloc();
    if (!frame) {
        fprintf(stderr, "Could not allocate frame\n");
        printf("ffmpeg Audio Capture Thread:ERROR:001:thread exit!\n");
        return NULL;
    }

    oframe = av_frame_alloc();
    if (!oframe) {
        fprintf(stderr, "Could not allocate filtered frame\n");
        printf("ffmpeg Audio Capture Thread:ERROR:031:thread exit!\n");
        return NULL;
    }

    fprintf(stderr, "AA:audio_codec_ctx->channel_layout: %ld AV_CH_LAYOUT_STEREO: %lld default: %ld\n",
        global_audio_codec_ctx->channel_layout, (long long)AV_CH_LAYOUT_STEREO,
        av_get_default_channel_layout(global_audio_codec_ctx->channel_layout));

    if (global_audio_codec_ctx->channel_layout == 0)
    {
        // HINT: no idea what to do here. just guess STEREO?
        global_audio_codec_ctx->channel_layout = AV_CH_LAYOUT_STEREO;
    }

    fprintf(stderr, "AA:audio_codec_ctx->frame_size: %d\n",
        formatContext_audio->streams[audio_stream_index]->codecpar->frame_size);

    fprintf(stderr, "AA:audio_codec_ctx->sample_rate: %d\n",
        formatContext_audio->streams[audio_stream_index]->codecpar->sample_rate);


    SwrContext *swr_ctx = swr_alloc_set_opts(NULL,
                                 out_channel_layout, AV_SAMPLE_FMT_S16, out_sample_rate,
                                 global_audio_codec_ctx->channel_layout,
                                 formatContext_audio->streams[audio_stream_index]->codecpar->format,
                                 formatContext_audio->streams[audio_stream_index]->codecpar->sample_rate,
                                 0, NULL);
    if (!swr_ctx) {
        fprintf(stderr, "AA:Could not allocate resampler context\n");
        fprintf(stderr, "AA:%d %d %d %ld %d %d\n", (int)out_channel_layout,
                AV_SAMPLE_FMT_S16, out_sample_rate, global_audio_codec_ctx->channel_layout,
                global_audio_codec_ctx->sample_fmt, global_audio_codec_ctx->sample_rate);
        av_frame_free(&frame);
        return NULL;
    }

    if (swr_init(swr_ctx) < 0) {
        fprintf(stderr, "AA:Could not initialize resampler context\n");
        fprintf(stderr, "AA:%d %d %d %ld %d %d\n", (int)out_channel_layout,
                AV_SAMPLE_FMT_S16, out_sample_rate, global_audio_codec_ctx->channel_layout,
                global_audio_codec_ctx->sample_fmt, global_audio_codec_ctx->sample_rate);
        av_frame_free(&frame);
        swr_free(&swr_ctx);
        return NULL;
    }

    fprintf(stderr, "AA:Audio Config:%d %d %d %ld %d %d\n", (int)out_channel_layout,
            AV_SAMPLE_FMT_S16, out_sample_rate, global_audio_codec_ctx->channel_layout,
            global_audio_codec_ctx->sample_fmt, global_audio_codec_ctx->sample_rate);

    JNIEnv *jnienv2 = NULL;
    if (jnienv2 == NULL)
    {
        JavaVMAttachArgs args;
        args.version = JNI_VERSION_1_6; // choose your JNI version
        args.name = "t_ff_jacb"; // you might want to give the java thread a name
        args.group = NULL; // you might want to assign the java thread to a ThreadGroup
        if (cachedJVM)
        {
            (*cachedJVM)->AttachCurrentThread(cachedJVM, (void **) &jnienv2, &args);
        }
    }

    fifo_buffer_t* audio_pcm_buffer = fifo_buffer_create(temp_audio_buf_sizes);

    // ------------ create filter ------------
    AVFilterContext *abuffer_ctx = NULL;
    AVFilterContext *aformat_ctx = NULL;
    AVFilterContext *abuffersink_ctx = NULL;
    AVFilterGraph *filter_graph = NULL;
    filter_graph = avfilter_graph_alloc();
    if (!filter_graph) {
        fprintf(stderr, "ERROR: unable to create filter graph\n");
    } else {

        // ---------- here is the actual filter ----------
        AVFilter *noisefilter = avfilter_get_by_name("arnndn");
        AVFilterContext *noisefilter_ctx = NULL;
        static char args_strbuf[1512];
        if (resources_dir_path) {
            snprintf(args_strbuf, sizeof(args_strbuf), "m=%smp.rnnn", resources_dir_path);
        } else {
            snprintf(args_strbuf, sizeof(args_strbuf), "m=mp.rnnn");
        }
        fprintf(stderr, "afftdn filter: %s\n", args_strbuf);

        /*
        // -------- afftdn filter --------
        AVFilter *noisefilter = avfilter_get_by_name("afftdn");
        AVFilterContext *noisefilter_ctx = NULL;
        static char strbuf[512];
        snprintf(strbuf, sizeof(strbuf), "nt=w:om=output");
        fprintf(stderr, "afftdn filter: %s\n", strbuf);
        */
        /*
        // -------- volume filter --------
        AVFilter *noisefilter = avfilter_get_by_name("volume");
        AVFilterContext *noisefilter_ctx = NULL;
        static char strbuf[512];
        double vol = 0.20;
        snprintf(strbuf, sizeof(strbuf), "volume=%f", vol);
        fprintf(stderr, "volume: %s\n", strbuf);
        */
        // -------- volume filter --------
        int err_filter = avfilter_graph_create_filter(&noisefilter_ctx, noisefilter, NULL, args_strbuf, NULL, filter_graph);
        if (err_filter < 0) {
            fprintf(stderr, "ERROR: error initializing noise filter\n");
            filter_init_error = 1;
        }
        // ---------- here is the actual filter ----------

        if (filter_init_error == 1)
        {
            avfilter_graph_free(&filter_graph);
        }
        else
        {
            // ---------- abuffer ----------
            AVFilter *abuffer = avfilter_get_by_name("abuffer");
            memset(args_strbuf, 0, sizeof(args_strbuf));
            snprintf(args_strbuf, sizeof(args_strbuf),
                    "sample_rate=%d:sample_fmt=%s:channel_layout=0x%"PRIx64,
                    global_audio_codec_ctx->sample_rate,
                    av_get_sample_fmt_name(global_audio_codec_ctx->sample_fmt),
                    global_audio_codec_ctx->channel_layout
                    );
            fprintf(stderr, "abuffer: %s\n", args_strbuf);
            int err = avfilter_graph_create_filter(&abuffer_ctx, abuffer, NULL, args_strbuf, NULL, filter_graph);
            if (err < 0) {
                fprintf(stderr, "ERROR: error initializing abuffer filter\n");
            }
            // ---------- abuffer ----------

            // ---------- aformat ----------
            /* Create the aformat filter;
            * it ensures that the output is of the format we want. */
            AVFilter *aformat = avfilter_get_by_name("aformat");
            if (!aformat) {
                fprintf(stderr, "ERROR: Could not find the aformat filter\n");
            }
            aformat_ctx = avfilter_graph_alloc_filter(filter_graph, aformat, "aformat");
            if (!aformat_ctx) {
                fprintf(stderr, "ERROR: Could not allocate the aformat instance.\n");
            }
            /* passing the options is in a string of the form
            * key1=value1:key2=value2.... */
            memset(args_strbuf, 0, sizeof(args_strbuf));
            snprintf(args_strbuf, sizeof(args_strbuf),
                    "sample_fmts=%s:sample_rates=%d:channel_layouts=0x%"PRIx64,
                    av_get_sample_fmt_name(global_audio_codec_ctx->sample_fmt),
                    global_audio_codec_ctx->sample_rate,
                    global_audio_codec_ctx->channel_layout
                    );
            err = avfilter_init_str(aformat_ctx, args_strbuf);
            if (err < 0) {
                fprintf(stderr, "ERROR: Could not initialize the aformat filter.\n");
            }
            // ---------- aformat ----------

            // ---------- abuffersink ----------
            AVFilter *abuffersink = avfilter_get_by_name("abuffersink");
            int err_abuffersink = avfilter_graph_create_filter(&abuffersink_ctx, abuffersink,
                    NULL, NULL, NULL, filter_graph);
            if (err_abuffersink < 0) {
                fprintf(stderr, "ERROR: error initializing abuffer filter\n");
            }
            // ---------- abuffersink ----------

            // *************************
            //
            // input ==> abuffer_ctx -> noisefilter_ctx -> aformat_ctx -> abuffersink_ctx ==> output
            //
            if (err >= 0) err = avfilter_link(abuffer_ctx, 0, noisefilter_ctx, 0);
            if (err >= 0) err = avfilter_link(noisefilter_ctx, 0, aformat_ctx, 0);
            if (err >= 0) err = avfilter_link(aformat_ctx, 0, abuffersink_ctx, 0);
            if (err < 0) {
                fprintf(stderr, "ERROR: error connecting filters\n");
            }

            int err_fgraph = avfilter_graph_config(filter_graph, NULL);
            if (err_fgraph < 0) {
                fprintf(stderr, "ERROR: error configuring the filter graph\n");
                avfilter_graph_free(&filter_graph);
            } else {
                use_filter = 1;
                fprintf(stderr, "OK: using filter\n");
            }
        }
    }
    // ------------ create filter ------------

    while (global_audio_in_capture_running == true)
    {
        if (av_read_frame(formatContext_audio, &packet) >= 0)
        {
            if (packet.stream_index == audio_stream_index)
            {
                // Decode audio packet
                ret = avcodec_send_packet(global_audio_codec_ctx, &packet);
                if (ret < 0)
                {
                    fprintf(stderr, "Error sending audio packet for decoding\n");
                    break;
                }
                while (ret >= 0)
                {
                    ret = avcodec_receive_frame(global_audio_codec_ctx, frame);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    {
                        break;
                    }
                    else if (ret < 0)
                    {
                        fprintf(stderr, "Error during audio decoding\n");
                        break;
                    }

                    if ((use_filter == 1) && (apply_audio_filter == 1))
                    {
                        // push the audio data from decoded frame into the filtergraph
                        int err_filter_in = av_buffersrc_write_frame(abuffer_ctx, frame);
                        if (err_filter_in < 0) {
                            fprintf(stderr, "ERROR: error writing audio frame to buffersrc\n");
                        } else {
                            // fprintf(stderr, "OK: writing audio frame to buffersrc\n");
                        }
                        // run USING the filter -----------------------
                        for (;;) {
                            int err = av_buffersink_get_frame(abuffersink_ctx, oframe);
                            if (err == AVERROR_EOF || err == AVERROR(EAGAIN))
                                break;
                            if (err < 0) {
                                fprintf(stderr, "ERROR: error reading buffer from buffersink\n");
                                break;
                            }

                            /*
                            fprintf(stderr, "frame: %d %d %d %s %s\n",
                                    frame->nb_samples,
                                    (int)frame->channel_layout,
                                    frame->channels,
                                    av_get_sample_fmt_name(frame->format),
                                    av_get_sample_fmt_name(global_audio_codec_ctx->sample_fmt));

                            fprintf(stderr, "oframe: %d %d %d %s\n",
                                    oframe->nb_samples,
                                    (int)oframe->channel_layout,
                                    oframe->channels,
                                    av_get_sample_fmt_name(oframe->format));
                            */
                            num_samples = av_rescale_rnd(oframe->nb_samples,
                                                                out_sample_rate,
                                                                global_audio_codec_ctx->sample_rate,
                                                                AV_ROUND_UP);

                            av_samples_alloc_array_and_samples(&converted_samples,
                                        NULL,
                                        out_channels,
                                        num_samples,
                                        AV_SAMPLE_FMT_S16,
                                        0);
                            __attribute__((unused)) int samples_out = swr_convert(swr_ctx,
                                    converted_samples,
                                    num_samples,
                                    (const uint8_t **)oframe->extended_data,
                                    oframe->nb_samples);

                            const int want_write_bytes = (num_samples * out_channels * out_bytes_per_sample);
                            __attribute__((unused)) size_t written_bytes = fifo_buffer_write(audio_pcm_buffer, converted_samples[0], want_write_bytes);

                            audio_delay_in_bytes = (out_samples * out_channels * out_bytes_per_sample) * global_audio_delay_factor; // n x 60 ms delay
                            if (fifo_buffer_data_available(audio_pcm_buffer) >= (out_samples * out_channels * out_bytes_per_sample) + (audio_delay_in_bytes))
                            {
                                const int data_size_bytes = out_samples * out_channels * out_bytes_per_sample;
                                if (audio_buffer_pcm_2_size >= (long)data_size_bytes)
                                {
                                    size_t read_bytes = fifo_buffer_read(
                                            audio_pcm_buffer,
                                            audio_buffer_pcm_2,
                                            data_size_bytes);

                                    if (jnienv2 != NULL) {
                                    (*jnienv2)->CallStaticVoidMethod(jnienv2, AVActivity,
                                            callback_audio_capture_frame_pts_cb_method,
                                            (jlong)read_bytes,
                                            (jint)out_samples,
                                            (jint)out_channels,
                                            (jint)out_sample_rate,
                                            (jlong)0
                                        );
                                    }
                                    else
                                    {
                                        fprintf(stderr, "could not attach thread to JVM [3]\n");
                                    }
                                }
                                else
                                {
                                    fprintf(stderr, "JNI audio caputre buffer too small\n");
                                    if (jnienv2 != NULL) {
                                    (*jnienv2)->CallStaticVoidMethod(jnienv2, AVActivity,
                                            callback_audio_capture_frame_too_small_cb_method,
                                            (jint)data_size_bytes
                                        );
                                    }
                                    else
                                    {
                                        fprintf(stderr, "could not attach thread to JVM [4]\n");
                                    }
                                }
                            }

                            av_freep(&converted_samples[0]);
                            av_freep(&converted_samples);
                        }
                        // run USING the filter -----------------------
                    }
                    else
                    {
                        // run without filter -----------------------
                        num_samples = av_rescale_rnd(frame->nb_samples,
                                                            out_sample_rate,
                                                            global_audio_codec_ctx->sample_rate,
                                                            AV_ROUND_UP);

                        av_samples_alloc_array_and_samples(&converted_samples,
                                    NULL,
                                    out_channels,
                                    num_samples,
                                    AV_SAMPLE_FMT_S16,
                                    0);
                        __attribute__((unused)) int samples_out = swr_convert(swr_ctx,
                                converted_samples,
                                num_samples,
                                (const uint8_t **)frame->extended_data,
                                frame->nb_samples);

                        const int want_write_bytes = (num_samples * out_channels * out_bytes_per_sample);
                        __attribute__((unused)) size_t written_bytes = fifo_buffer_write(audio_pcm_buffer, converted_samples[0], want_write_bytes);

                        audio_delay_in_bytes = (out_samples * out_channels * out_bytes_per_sample) * global_audio_delay_factor; // n x 60 ms delay
                        if (fifo_buffer_data_available(audio_pcm_buffer) >= (out_samples * out_channels * out_bytes_per_sample) + (audio_delay_in_bytes))
                        {
                            const int data_size_bytes = out_samples * out_channels * out_bytes_per_sample;
                            if (audio_buffer_pcm_2_size >= (long)data_size_bytes)
                            {
                                size_t read_bytes = fifo_buffer_read(
                                        audio_pcm_buffer,
                                        audio_buffer_pcm_2,
                                        data_size_bytes);

                                if (jnienv2 != NULL) {
                                (*jnienv2)->CallStaticVoidMethod(jnienv2, AVActivity,
                                        callback_audio_capture_frame_pts_cb_method,
                                        (jlong)read_bytes,
                                        (jint)out_samples,
                                        (jint)out_channels,
                                        (jint)out_sample_rate,
                                        (jlong)0
                                    );
                                }
                                else
                                {
                                    fprintf(stderr, "could not attach thread to JVM [5]\n");
                                }
                            }
                            else
                            {
                                fprintf(stderr, "JNI audio caputre buffer too small\n");
                                if (jnienv2 != NULL) {
                                (*jnienv2)->CallStaticVoidMethod(jnienv2, AVActivity,
                                        callback_audio_capture_frame_too_small_cb_method,
                                        (jint)data_size_bytes
                                    );
                                }
                                else
                                {
                                    fprintf(stderr, "could not attach thread to JVM [6]\n");
                                }
                            }
                        }

                        av_freep(&converted_samples[0]);
                        av_freep(&converted_samples);
                        // run without filter -----------------------
                    }
                }
            }
            av_packet_unref(&packet);
        }
        // yieldcpu(100);
    }

    av_frame_free(&frame);
    av_frame_free(&oframe);
    swr_free(&swr_ctx);

    // ------------ destroy filter ------------
    if (use_filter == 1) {
        avfilter_graph_free(&filter_graph);
    }
    // ------------ destroy filter ------------

    if (cachedJVM)
    {
        (*cachedJVM)->DetachCurrentThread(cachedJVM);
    }

    printf("ffmpeg Audio Capture Thread:Clean thread exit!\n");
    return NULL;
}

JNIEXPORT jstring JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1version(JNIEnv *env, jobject thiz)
{
    return (*env)->NewStringUTF(env, global_version_string);
}

JNIEXPORT jstring JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1libavutil_1version(JNIEnv *env, jobject thiz)
{
    char libavutil_version_str[2000];
    CLEAR(libavutil_version_str);
    snprintf(libavutil_version_str, 1999, "%d.%d.%d", (int)LIBAVUTIL_VERSION_MAJOR, (int)LIBAVUTIL_VERSION_MINOR, (int)LIBAVUTIL_VERSION_MICRO);
    return (*env)->NewStringUTF(env, libavutil_version_str);
}

JNIEXPORT void JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1apply_1audio_1filter(JNIEnv *env, jobject thiz, jint apply_filter)
{
    apply_audio_filter = apply_filter;
    fprintf(stderr, "apply_audio_filter=%d\n", apply_audio_filter);
}

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1init(JNIEnv *env, jobject thiz, jstring resources_dir)
{
    if (resources_dir == NULL)
    {
        if (resources_dir_path) {
            memset(resources_dir_path, 0, resources_dir_path_maxlen + 4);
        }
    }
    else
    {
        const char *resources_dir_cstr = (*env)->GetStringUTFChars(env, resources_dir, NULL);
        if (resources_dir_cstr != NULL)
        {
            if (!resources_dir_path) {
                resources_dir_path = calloc(1, resources_dir_path_maxlen + 4);
            } else {
                memset(resources_dir_path, 0, resources_dir_path_maxlen + 4);
            }
            if (resources_dir_path)
            {
                snprintf(resources_dir_path, resources_dir_path_maxlen, "%s", resources_dir_cstr);
                fprintf(stderr, "init: resources_dir_path=%s\n", resources_dir_path);
            }
            (*env)->ReleaseStringUTFChars(env, resources_dir, resources_dir_cstr);
        }
    }

#if (LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58,9,100))
    av_register_all();
#endif
    // avformat_network_init();
    avdevice_register_all();

    jclass cls_local = (*env)->GetObjectClass(env, thiz);
    java_find_class_global("com/zoffcc/applications/ffmpegav/AVActivity", &AVActivity);

    printf("cls_local=%p\n", cls_local);
    printf("AVActivity=%p\n", AVActivity);

    if (pthread_mutex_init(&vsend___mutex, NULL) != 0)
    {
        fprintf(stderr, "Creating vsend mutex failed\n");
        // HINT: add error handling
        // return -1;
    }

    if (pthread_mutex_init(&vscale___mutex, NULL) != 0)
    {
        fprintf(stderr, "Creating vscale mutex failed\n");
        // HINT: add error handling
        // return -1;
    }

    if (pthread_mutex_init(&vbuffer2___mutex, NULL) != 0)
    {
        fprintf(stderr, "Creating vbuffer2 mutex failed\n");
        // HINT: add error handling
        // return -1;
    }

    callback_video_capture_frame_pts_cb_method = (*env)->GetStaticMethodID(env, AVActivity,
            "ffmpegav_callback_video_capture_frame_pts_cb_method", "(JJJJJII)V");
    printf("ffmpegav_callback_video_capture_frame_pts_cb_method=%p\n", callback_video_capture_frame_pts_cb_method);

    callback_video_capture_frame_too_small_cb_method = (*env)->GetStaticMethodID(env, AVActivity,
            "ffmpegav_callback_video_capture_frame_too_small_cb_method", "(III)V");
    printf("ffmpegav_callback_video_capture_frame_too_small_cb_method=%p\n", callback_video_capture_frame_too_small_cb_method);

    callback_audio_capture_frame_pts_cb_method = (*env)->GetStaticMethodID(env, AVActivity,
            "ffmpegav_callback_audio_capture_frame_pts_cb_method", "(JIIIJ)V");
    printf("ffmpegav_callback_audio_capture_frame_pts_cb_method=%p\n", callback_audio_capture_frame_pts_cb_method);

    callback_audio_capture_frame_too_small_cb_method = (*env)->GetStaticMethodID(env, AVActivity,
            "ffmpegav_callback_audio_capture_frame_too_small_cb_method", "(I)V");
    printf("ffmpegav_callback_audio_capture_frame_too_small_cb_method=%p\n", callback_audio_capture_frame_too_small_cb_method);

    fprintf(stderr, "init: DONE\n");
    // HINT: add error handling
    return 0;
}

JNIEXPORT jobjectArray JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1get_1in_1sources(JNIEnv *env, jobject thiz, jstring devicename, jint is_video)
{
    if (devicename == NULL)
    {
        return NULL;
    }
    const char *devicename_cstr = (*env)->GetStringUTFChars(env, devicename, NULL);
    if (devicename_cstr == NULL)
    {
        return NULL;
    }
    const uint32_t max_sources = 64;
    // add 1 more for macos "avfoundation" stuff that is not detected
    jobjectArray result = (*env)->NewObjectArray(env, (max_sources + 1), (*env)->FindClass(env, "java/lang/String"), NULL);

    printf("wanted_in_device=%s\n", devicename_cstr);
    AVInputFormat *inputFormat_search = av_find_input_format(devicename_cstr);
    AVDeviceInfoList *deviceInfoList = NULL;
    AVDeviceInfo *deviceInfo = NULL;
    int i;
    uint32_t in_source_count = 0;

#ifdef __linux__
    if (strncmp((char *)devicename_cstr, "x11grab", strlen((char *)"x11grab")) == 0)
    {
        // add available displays to "x11grag" input sources
        Display *display = XOpenDisplay(NULL);
        if (display != NULL)
        {
            int count_display = XScreenCount(display);
            printf("Displays counter: %d\n", count_display);
            if (count_display > 0)
            {
                char *current_display_string = XDisplayString(display);
                if (current_display_string != NULL)
                {
                    if (strlen(current_display_string) > 0)
                    {
                        printf("current Display string: %s\n", current_display_string);
                        jstring str = (*env)->NewStringUTF(env, current_display_string); // format = ":<n>"
                        (*env)->SetObjectArrayElement(env, result, in_source_count, str);
                        in_source_count++;
                    }
                }
            }
            XCloseDisplay(display);
        }
    }
#endif
    int devices_found = avdevice_list_input_sources(inputFormat_search, NULL, NULL, &deviceInfoList);
    printf("inputs found: %d\n", devices_found);
    if (devices_found < 1)
    {
        printf("inputs found: %d\n", devices_found);
        avdevice_free_list_devices(&deviceInfoList);
#ifdef __APPLE__
        if (strncmp(devicename_cstr, "avfoundation", strlen("avfoundation")) == 0) {
            if (is_video == 1) {
                jstring str1 = (*env)->NewStringUTF(env, "0:"); // format = "[VIDEO]:[AUDIO]"
                (*env)->SetObjectArrayElement(env, result, in_source_count, str1);
                in_source_count++;
                // HINT: add 3 more displays, in case that there are more displays attached
                jstring str2 = (*env)->NewStringUTF(env, "1:"); // format = "[VIDEO]:[AUDIO]"
                (*env)->SetObjectArrayElement(env, result, in_source_count, str2);
                in_source_count++;
                jstring str3 = (*env)->NewStringUTF(env, "2:"); // format = "[VIDEO]:[AUDIO]"
                (*env)->SetObjectArrayElement(env, result, in_source_count, str3);
                in_source_count++;
                jstring str4 = (*env)->NewStringUTF(env, "3:"); // format = "[VIDEO]:[AUDIO]"
                (*env)->SetObjectArrayElement(env, result, in_source_count, str4);
                in_source_count++;
            } else {
                jstring str = (*env)->NewStringUTF(env, ":0"); // format = "[VIDEO]:[AUDIO]"
                (*env)->SetObjectArrayElement(env, result, in_source_count, str);
                in_source_count++;
            }
            (*env)->ReleaseStringUTFChars(env, devicename, devicename_cstr);
            return result;
        }
        (*env)->ReleaseStringUTFChars(env, devicename, devicename_cstr);
        return NULL;
#else
    #ifdef __MINGW32__
        if (strncmp(devicename_cstr, "gdigrab", strlen("gdigrab")) == 0) {
            if (is_video == 1) {
                jstring str = (*env)->NewStringUTF(env, "desktop");
                (*env)->SetObjectArrayElement(env, result, in_source_count, str);
                (*env)->ReleaseStringUTFChars(env, devicename, devicename_cstr);
                return result;
            }
        }
        (*env)->ReleaseStringUTFChars(env, devicename, devicename_cstr);
        return NULL;
    #else
        (*env)->ReleaseStringUTFChars(env, devicename, devicename_cstr);
        if (in_source_count > 0)
        {
            // there are manually added/detected devices
            // return "result"
            printf("return manual result. devices: %d\n", in_source_count);
            return result;
        }
        else
        {
            return NULL;
        }
    #endif
#endif
    }

    for (i = 0; i < deviceInfoList->nb_devices; i++) {
        deviceInfo = deviceInfoList->devices[i];
        if (deviceInfo->device_name)
        {
            printf("input #%d: %s\n", i, deviceInfo->device_name);
            if (deviceInfo->device_description != NULL) {
                printf("input descr. #%d: %s\n", i, deviceInfo->device_description);
            }
            jstring str = (*env)->NewStringUTF(env, deviceInfo->device_name);
            (*env)->SetObjectArrayElement(env, result, in_source_count, str);
            in_source_count++;
            if (in_source_count >= max_sources)
            {
                break;
            }
        }
    }

#ifdef __APPLE__
    if (strncmp(devicename_cstr, "avfoundation", strlen("avfoundation")) == 0) {
        if (is_video == 1) {
            jstring str = (*env)->NewStringUTF(env, "0:"); // format = "[VIDEO]:[AUDIO]"
            (*env)->SetObjectArrayElement(env, result, in_source_count, str);
        } else {
            jstring str = (*env)->NewStringUTF(env, ":0"); // format = "[VIDEO]:[AUDIO]"
            (*env)->SetObjectArrayElement(env, result, in_source_count, str);
        }
    }
#endif

    avdevice_free_list_devices(&deviceInfoList);
    (*env)->ReleaseStringUTFChars(env, devicename, devicename_cstr);
    return result;
}

JNIEXPORT jobjectArray JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1get_1video_1in_1devices(JNIEnv *env, jobject thiz)
{
    const uint32_t max_devices = 64;
    jobjectArray result = (*env)->NewObjectArray(env, max_devices, (*env)->FindClass(env, "java/lang/String"), NULL);
    AVInputFormat *inputFormat_search = av_input_video_device_next(NULL);
    uint32_t in_device_count = 0;
    while (inputFormat_search != NULL) {
        // printf("Input Device Name: %s long name: %s\n", inputFormat_search->name, inputFormat_search->long_name);
        jstring str = (*env)->NewStringUTF(env, inputFormat_search->name);
        (*env)->SetObjectArrayElement(env, result, in_device_count, str);
        in_device_count++;
        if (in_device_count >= max_devices)
        {
            break;
        }
        inputFormat_search = av_input_video_device_next(inputFormat_search);
        if (inputFormat_search == NULL)
        {
            break;
        }
    }

#ifdef __APPLE__
    if (in_device_count == 0) {
        jstring str = (*env)->NewStringUTF(env, "avfoundation");
        (*env)->SetObjectArrayElement(env, result, in_device_count, str);
    }
#endif
#ifdef __MINGW32__
    if (in_device_count == 0) {
        jstring str = (*env)->NewStringUTF(env, "dshow");
        (*env)->SetObjectArrayElement(env, result, in_device_count, str);
    }

#endif

    return result;
}

JNIEXPORT jobjectArray JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1get_1audio_1in_1devices(JNIEnv *env, jobject thiz)
{
    const uint32_t max_devices = 64;
    jobjectArray result = (*env)->NewObjectArray(env, max_devices, (*env)->FindClass(env, "java/lang/String"), NULL);
    AVInputFormat *inputFormat_search = av_input_audio_device_next(NULL);
    uint32_t in_device_count = 0;
    while (inputFormat_search != NULL) {
        // printf("Input Device Name: %s long name: %s\n", inputFormat_search->name, inputFormat_search->long_name);
        jstring str = (*env)->NewStringUTF(env, inputFormat_search->name);
        (*env)->SetObjectArrayElement(env, result, in_device_count, str);
        in_device_count++;
        if (in_device_count >= max_devices)
        {
            break;
        }
        inputFormat_search = av_input_audio_device_next(inputFormat_search);
        if (inputFormat_search == NULL)
        {
            break;
        }
    }

#ifdef __APPLE__
    if (in_device_count == 0) {
        jstring str = (*env)->NewStringUTF(env, "avfoundation");
        (*env)->SetObjectArrayElement(env, result, in_device_count, str);
    }
#endif
#ifdef __MINGW32__
    if (in_device_count == 0) {
        jstring str = (*env)->NewStringUTF(env, "dshow");
        (*env)->SetObjectArrayElement(env, result, in_device_count, str);
    }

#endif

    return result;
}

/**
 * @brief Prints video codec parameters
 *
 * @param codecpar AVCodecParameters pointer to the video codec parameters
 * @param text_prefix Prefix to add to the printed text
 */
static void print_codec_parameters_video(AVCodecParameters *codecpar, const char* text_prefix) {
    AVCodecContext *codec = avcodec_alloc_context3(NULL);
    if (codec == NULL)
    {
        return;
    }
    printf("%s===================================\n", text_prefix);
    int res = avcodec_parameters_to_context(codec, codecpar);
    if (res >= 0)
    {
        printf("%sCodec Type: %s\n", text_prefix, avcodec_get_name(codec->codec_id));
        if ((codec->codec != NULL) && (av_get_profile_name(codec->codec, codec->profile) != NULL))
        {
            printf("%sCodec Profile: %s\n", text_prefix, av_get_profile_name(codec->codec, codec->profile));
        }
        printf("%sCodec Level: %d\n", text_prefix, codec->level);
        printf("%sCodec Width: %d\n", text_prefix, codec->width);
        printf("%sCodec Height: %d\n", text_prefix, codec->height);
        printf("%sCodec Bitrate: %ld\n", text_prefix, codec->bit_rate);
        if (av_get_pix_fmt_name(codec->pix_fmt) != NULL)
        {
            printf("%sCodec Format: %s\n", text_prefix, av_get_pix_fmt_name(codec->pix_fmt));
        }
    }
    printf("%s===================================\n", text_prefix);
    avcodec_free_context(&codec);
}

/**
 * @brief Prints audio codec parameters
 *
 * @param codecpar AVCodecParameters pointer to the audio codec parameters
 * @param text_prefix Prefix to add to the printed text
 */
static void print_codec_parameters_audio(AVCodecParameters *codecpar, const char* text_prefix) {
    printf("%s===================================\n", text_prefix);
    printf("%sCodec Type: %s\n", text_prefix, avcodec_get_name(codecpar->codec_id));
    printf("%sCodec ID: %d\n", text_prefix, codecpar->codec_id);
    if (av_get_sample_fmt_name(codecpar->format) != NULL)
    {
        printf("%sSample Format: %s\n", text_prefix, av_get_sample_fmt_name(codecpar->format));
    }
    printf("%sSample Rate: %d\n", text_prefix, codecpar->sample_rate);
    printf("%sChannels: %d\n", text_prefix, codecpar->channels);
    printf("%sChannel Layout: %ld\n", text_prefix, codecpar->channel_layout);
    printf("%sBit Rate: %ld\n", text_prefix, codecpar->bit_rate);
    printf("%sBlock Align: %d\n", text_prefix, codecpar->block_align);
    printf("%sFrame Size: %d\n", text_prefix, codecpar->frame_size);
    printf("%sInitial Padding: %d\n", text_prefix, codecpar->initial_padding);
    printf("%sTrailing Padding: %d\n", text_prefix, codecpar->trailing_padding);
    printf("%sSeek Preroll: %d\n", text_prefix, codecpar->seek_preroll);
    printf("%s===================================\n", text_prefix);
}

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1open_1video_1in_1device(JNIEnv *env, jobject thiz,
    jstring deviceformat, jstring inputname,
    jint wanted_width, jint wanted_height, jint fps, jint force_mjpeg)
{
    if (deviceformat == NULL)
    {
        return -1;
    }
    if (inputname == NULL)
    {
        return -1;
    }
    const char *deviceformat_cstr = (*env)->GetStringUTFChars(env, deviceformat, NULL);
    if (deviceformat_cstr == NULL)
    {
        return -1;
    }
    printf("wanted_video_in_device=%s\n", deviceformat_cstr);
    const char *inputname_cstr = (*env)->GetStringUTFChars(env, inputname, NULL);
    if (inputname_cstr == NULL)
    {
        return -1;
    }
    printf("wanted_video_in_input=%s\n", inputname_cstr);

    fprintf(stderr, "wanted_video_capture_resolution: %dx%d\n", wanted_width, wanted_height);

    output_width = wanted_width;
    output_height = wanted_height;

    inputFormat_video = av_find_input_format(deviceformat_cstr);
    if (!inputFormat_video) {
        fprintf(stderr, "Could not find input format\n");
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
    }

    if (strncmp((char *)deviceformat_cstr, "x11grab", strlen((char *)"x11grab")) == 0)
    {
#ifdef __linux__
        // capture desktop on X11 Linux
        Display *display = XOpenDisplay(NULL);
        if (display == NULL)
        {
            fprintf(stderr, "Could not find X11 Display.\n");
            reset_video_in_values();
            (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
            (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
            return -1;
        }
        Window root = DefaultRootWindow(display);
        XWindowAttributes attributes;
        XGetWindowAttributes(display, root, &attributes);
        int screen_width = attributes.width;
        int screen_height = attributes.height;
        XCloseDisplay(display);

        fprintf(stderr, "Display Screen: %dx%d\n", screen_width, screen_height);

        if ((screen_width < 16) || (screen_width > 10000))
        {
            screen_width = 640;
        }

        if ((screen_height < 16) || (screen_height > 10000))
        {
            screen_height = 480;
        }

        fprintf(stderr, "Display Screen corrected: %dx%d\n", screen_width, screen_height);
        char fps_str[20];
        CLEAR(fps_str);
        snprintf(fps_str, 4, "%d", fps);
        char *capture_fps = fps_str;
        fprintf(stderr, "Display Screen capture FPS: %s\n", capture_fps);

        AVDictionary* options = NULL;
        // set some options ----------------------
        //
        // grabbing frame rate
        av_dict_set(&options, "framerate", capture_fps, 0);
        //
        // TODO: make this an option
        // do not capture the mouse pointer
        av_dict_set(&options, "draw_mouse", "0", 0);
        //
        // make the grabbed area follow the mouse
        // av_dict_set(&options, "follow_mouse", "centered", 0);
        //
        // set some options ----------------------

        const int resolution_string_len = 1000;
        char resolution_string[resolution_string_len];
        memset(resolution_string, 0, resolution_string_len);
        snprintf(resolution_string, resolution_string_len, "%dx%d", screen_width, screen_height);
        fprintf(stderr, "Display resolution_string: %s\n", resolution_string);
        av_dict_set(&options, "video_size", resolution_string, 0);
        av_dict_set(&options, "probesize", "50M", 0);

        const int desktop_display_cap_str_len = 1000;
        char desktop_display_cap_str[desktop_display_cap_str_len];
        memset(desktop_display_cap_str, 0, desktop_display_cap_str_len);
        snprintf(desktop_display_cap_str, desktop_display_cap_str_len, "%s+0,0", inputname_cstr);
        fprintf(stderr, "Display capture_string: %s\n", desktop_display_cap_str);

        // example: grab at position 10,20 ":0.0+10,20"
        if (avformat_open_input(&formatContext_video, desktop_display_cap_str, inputFormat_video, &options) != 0)
        {
            fprintf(stderr, "Could not open desktop as video input stream.\n");
            reset_video_in_values();
            (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
            (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
            return -1;
        }
#else
        fprintf(stderr, "Could not open desktop as video input stream.\n");
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
#endif
    }
    else if (strncmp((char *)deviceformat_cstr, "video4linux2", strlen((char *)"video4linux2")) == 0)
    {
#ifdef __linux__
        AVDictionary* options = NULL;

        const int camresolution_string_len = 1000;
        char camresolution_string[camresolution_string_len];
        memset(camresolution_string, 0, camresolution_string_len);
        snprintf(camresolution_string, camresolution_string_len, "%dx%d", wanted_width, wanted_height);
        fprintf(stderr, "Wanted camera resolution_string: %s\n", camresolution_string);

        av_dict_set(&options, "video_size", camresolution_string, 0);
        if (force_mjpeg == 1) {
            av_dict_set(&options, "input_format", "mjpeg", 0);
        }
        // av_dict_set(&options, "framerate", "15", 0);
        if (avformat_open_input(&formatContext_video, inputname_cstr, inputFormat_video, &options) != 0)
        {
            fprintf(stderr, "Could not open desktop as video input stream.\n");
            reset_video_in_values();
            (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
            (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
            return -1;
        }
#else
        fprintf(stderr, "Could not open v4l2 as video input stream.\n");
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
#endif
    }
    else if (avformat_open_input(&formatContext_video, inputname_cstr, inputFormat_video, &options_video) < 0) {
        fprintf(stderr, "Could not open input\n");
#ifdef __MINGW32__
        // try again with "video=...." prepended
        const char prefix[] = "video=";
        const uint32_t max_name_len = strlen(inputname_cstr) + strlen(prefix) + 1; // add 1 for the NULL character at the end!
        char *inputname_cstr_with_video_prepended = calloc(1, max_name_len + 8);
        if (inputname_cstr_with_video_prepended)
        {
            snprintf(inputname_cstr_with_video_prepended, max_name_len, "%s%s", prefix, inputname_cstr);
            fprintf(stderr, "trying to open: \"%s\"\n", inputname_cstr_with_video_prepended);
            if (avformat_open_input(&formatContext_video, inputname_cstr_with_video_prepended, inputFormat_video, &options_video) < 0) {
                fprintf(stderr, "Could not open input\n");
                reset_video_in_values();
                (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
                (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
                free(inputname_cstr_with_video_prepended);
                return -1;
            }
            free(inputname_cstr_with_video_prepended);
        }
        else
        {
            fprintf(stderr, "Could not allocate inputname with prefix\n");
            reset_video_in_values();
            (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
            (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
            return -1;
        }
#else
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
#endif
    }

    if (avformat_find_stream_info(formatContext_video, NULL) < 0) {
        fprintf(stderr, "Could not find stream info\n");
        avformat_close_input(&formatContext_video);
        av_dict_free(&options_video);
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
    }

    for (int i = 0; i < formatContext_video->nb_streams; i++) {
        AVCodecParameters *codec_params = formatContext_video->streams[i]->codecpar;
        AVCodecDescriptor *cdesc = avcodec_descriptor_get(codec_params->codec_id);
        if (cdesc != NULL)
        {
            fprintf(stderr, "needed codec: %d name: %s\n", codec_params->codec_id, cdesc->name);
        }
        else
        {
            fprintf(stderr, "needed codec: %d\n", codec_params->codec_id);
        }
        AVCodec *codec = avcodec_find_decoder(codec_params->codec_id);
        if (!codec)
        {
            fprintf(stderr, "Unsupported codec!\n");
            continue;
        }

        int ret;
        if (codec_params->codec_type == AVMEDIA_TYPE_VIDEO && video_stream_index < 0)
        {
            AVCodecContext *video_codec_ctx = avcodec_alloc_context3(codec);
            if (!video_codec_ctx) {
                fprintf(stderr, "Could not allocate video codec context\n");
                avcodec_free_context(&video_codec_ctx);
                break; // AVERROR(ENOMEM);
            }
            if ((ret = avcodec_parameters_to_context(video_codec_ctx, codec_params)) < 0) {
                fprintf(stderr, "Could not copy video codec parameters to context\n");
                avcodec_free_context(&video_codec_ctx);
                break; // ret;
            }
            if ((ret = avcodec_open2(video_codec_ctx, codec, NULL)) < 0) {
                fprintf(stderr, "Could not open video codec\n");
                avcodec_free_context(&video_codec_ctx);
                break; // ret;
            }
            video_stream_index = i;
            video_codec = codec;
            print_codec_parameters_video(codec_params, "VIDEO: ");
            time_base_video = formatContext_video->streams[i]->time_base;
            avcodec_free_context(&video_codec_ctx);
            global_video_codec_ctx = avcodec_alloc_context3(codec);
            avcodec_parameters_to_context(global_video_codec_ctx, codec_params);
            avcodec_open2(global_video_codec_ctx, codec, NULL);            
            break;
        }
    }

    if (video_stream_index < 0) {
        fprintf(stderr, "Could not find video stream\n");
        avformat_close_input(&formatContext_video);
        av_dict_free(&options_video);
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
    }

    (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
    (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1open_1audio_1in_1device(JNIEnv *env, jobject thiz,
    jstring deviceformat, jstring inputname)
{
    if (deviceformat == NULL)
    {
        return -1;
    }
    if (inputname == NULL)
    {
        return -1;
    }
    const char *deviceformat_cstr = (*env)->GetStringUTFChars(env, deviceformat, NULL);
    if (deviceformat_cstr == NULL)
    {
        return -1;
    }
    printf("wanted_audio_in_device=%s\n", deviceformat_cstr);

    const char *inputname_cstr = (*env)->GetStringUTFChars(env, inputname, NULL);
    if (inputname_cstr == NULL)
    {
        return -1;
    }
    printf("wanted_audio_in_input=%s\n", inputname_cstr);

    inputFormat_audio = av_find_input_format(deviceformat_cstr);
    if (!inputFormat_audio) {
        fprintf(stderr, "Could not find input format\n");
        reset_audio_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
    }

    if (strncmp((char *)deviceformat_cstr, "xxxxxx", strlen((char *)"xxxxxx")) == 0)
    {
#ifdef __linux__
#endif
    }
    else if (avformat_open_input(&formatContext_audio,
                inputname_cstr,
                inputFormat_audio, &options_audio) < 0) {
        fprintf(stderr, "Could not open input\n");
#ifdef __MINGW32__
        // try again with "audio=...." prepended
        const char prefix[] = "audio=";
        const uint32_t max_name_len = strlen(inputname_cstr) + strlen(prefix) + 1; // add 1 for the NULL character at the end!
        char *inputname_cstr_with_audio_prepended = calloc(1, max_name_len + 8);
        if (inputname_cstr_with_audio_prepended)
        {
            snprintf(inputname_cstr_with_audio_prepended, max_name_len, "%s%s", prefix, inputname_cstr);
            fprintf(stderr, "trying to open: \"%s\"\n", inputname_cstr_with_audio_prepended);
            if (avformat_open_input(&formatContext_audio, inputname_cstr_with_audio_prepended, inputFormat_audio, &options_audio) < 0) {
                fprintf(stderr, "Could not open input\n");
                reset_audio_in_values();
                (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
                (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
                free(inputname_cstr_with_audio_prepended);
                return -1;
            }
            free(inputname_cstr_with_audio_prepended);
        }
        else
        {
            fprintf(stderr, "Could not allocate inputname with prefix\n");
            reset_audio_in_values();
            (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
            (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
            return -1;
        }
#else
        reset_audio_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
#endif
    }

    if (avformat_find_stream_info(formatContext_audio, NULL) < 0) {
        fprintf(stderr, "Could not find stream info\n");
        avformat_close_input(&formatContext_audio);
        av_dict_free(&options_audio);
        reset_audio_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
    }

    for (int i = 0; i < formatContext_audio->nb_streams; i++) {
        AVCodecParameters *codec_params = formatContext_audio->streams[i]->codecpar;
        AVCodecDescriptor *cdesc = avcodec_descriptor_get(codec_params->codec_id);
        if (cdesc != NULL)
        {
            fprintf(stderr, "needed codec: %d name: %s\n", codec_params->codec_id, cdesc->name);
        }
        else
        {
            fprintf(stderr, "needed codec: %d\n", codec_params->codec_id);
        }
        AVCodec *codec = avcodec_find_decoder(codec_params->codec_id);
        if (!codec)
        {
            fprintf(stderr, "Unsupported codec!\n");
            continue;
        }

        int ret;
        if (codec_params->codec_type == AVMEDIA_TYPE_AUDIO && audio_stream_index < 0)
        {
            AVCodecContext *audio_codec_ctx = avcodec_alloc_context3(codec);
            if (!audio_codec_ctx) {
                fprintf(stderr, "Could not allocate audio codec context\n");
                avcodec_free_context(&audio_codec_ctx);
                break; // AVERROR(ENOMEM);
            }
            if ((ret = avcodec_parameters_to_context(audio_codec_ctx, codec_params)) < 0) {
                fprintf(stderr, "Could not copy audio codec parameters to context\n");
                avcodec_free_context(&audio_codec_ctx);
                break; // ret;
            }
            if ((ret = avcodec_open2(audio_codec_ctx, codec, NULL)) < 0) {
                fprintf(stderr, "Could not open audio codec\n");
                avcodec_free_context(&audio_codec_ctx);
                break; // ret;
            }
            audio_stream_index = i;
            audio_codec = codec;
            print_codec_parameters_audio(codec_params, "AUDIO: ");
            time_base_audio = formatContext_audio->streams[i]->time_base;
            avcodec_free_context(&audio_codec_ctx);
            global_audio_codec_ctx = avcodec_alloc_context3(codec);
            avcodec_parameters_to_context(global_audio_codec_ctx, codec_params);
            avcodec_open2(global_audio_codec_ctx, codec, NULL);
            break;
        }
    }

    if (audio_stream_index < 0) {
        fprintf(stderr, "Could not find audio stream\n");
        avformat_close_input(&formatContext_audio);
        av_dict_free(&options_audio);
        reset_audio_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
        return -1;
    }

    (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
    (*env)->ReleaseStringUTFChars(env, inputname, inputname_cstr);
    return 0;
}

JNIEXPORT jint JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1start_1video_1in_1capture(JNIEnv *env, jobject thiz)
{
    global_video_in_capture_running = true;
    // start capture thread
    if (pthread_create(&ffmpeg_thread_video_in_capture, NULL, ffmpeg_thread_video_in_capture_func, NULL) != 0)
    {
        printf("ffmpeg Video Capture Thread create failed\n");
    }
    else
    {
#ifdef __linux__
        pthread_setname_np(ffmpeg_thread_video_in_capture, "t_ff_vic");
#endif
        printf("ffmpeg Video Capture Thread successfully created\n");
        return 0;
    }
    return -1;
}

JNIEXPORT jint JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1start_1audio_1in_1capture(JNIEnv *env, jobject thiz)
{
    global_audio_in_capture_running = true;
    // start capture thread
    if (pthread_create(&ffmpeg_thread_audio_in_capture, NULL, ffmpeg_thread_audio_in_capture_func, NULL) != 0)
    {
        printf("ffmpeg Audio Capture Thread create failed\n");
    }
    else
    {
#ifdef __linux__
        pthread_setname_np(ffmpeg_thread_audio_in_capture, "t_ff_aic");
#endif
        printf("ffmpeg Audio Capture Thread successfully created\n");
        return 0;
    }
    return -1;
}

JNIEXPORT jint JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1stop_1audio_1in_1capture(JNIEnv *env, jobject thiz)
{
    global_audio_in_capture_running = false;
    // wait for capture thread to finish
    pthread_join(ffmpeg_thread_audio_in_capture, NULL);
    return 0;
}

JNIEXPORT jint JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1stop_1video_1in_1capture(JNIEnv *env, jobject thiz)
{
    global_video_in_capture_running = false;
    // wait for capture thread to finish
    pthread_join(ffmpeg_thread_video_in_capture, NULL);
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1close_1audio_1in_1device(JNIEnv *env, jobject thiz)
{
    if (global_audio_codec_ctx != NULL) {
        avcodec_free_context(&global_audio_codec_ctx);
    }
    global_audio_codec_ctx = NULL;
    if (formatContext_audio != NULL) {
        avformat_close_input(&formatContext_audio);
    }
    formatContext_audio = NULL;
    if (options_audio != NULL) {
        av_dict_free(&options_audio);
    }
    options_audio = NULL;
    audio_codec = NULL;
    audio_stream_index = -1;
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1close_1video_1in_1device(JNIEnv *env, jobject thiz)
{
    if (global_video_codec_ctx != NULL) {
        avcodec_free_context(&global_video_codec_ctx);
    }
    global_video_codec_ctx = NULL;
    if (formatContext_video != NULL) {
        avformat_close_input(&formatContext_video);
    }
    formatContext_video = NULL;
    if (options_video != NULL) {
        av_dict_free(&options_video);
    }
    options_video = NULL;
    video_codec = NULL;
    video_stream_index = -1;
    return 0;
}

// buffer is for playing video
JNIEXPORT void JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1set_1JNI_1video_1buffer
  (JNIEnv *env, jobject thiz, jobject recv_vbuf, jint frame_width_px, jint frame_height_px)
{
    JNIEnv *jnienv2;
    jnienv2 = jni_getenv();
    video_buffer_1 = (uint8_t *)(*jnienv2)->GetDirectBufferAddress(jnienv2, recv_vbuf);
    jlong capacity = (*jnienv2)->GetDirectBufferCapacity(jnienv2, recv_vbuf);
    video_buffer_1_size = (long)capacity;
    video_buffer_1_width = (int)frame_width_px;
    video_buffer_1_height = (int)frame_height_px;
    video_buffer_1_y_size = (int)(frame_width_px * frame_height_px);
    video_buffer_1_u_size = (int)(video_buffer_1_y_size / 4);
    video_buffer_1_v_size = (int)(video_buffer_1_y_size / 4);
    video_buffer_1_u = (uint8_t *)(video_buffer_1 + video_buffer_1_y_size);
    video_buffer_1_v = (uint8_t *)(video_buffer_1 + video_buffer_1_y_size + video_buffer_1_u_size);
}

// buffer2 is for capturing video
JNIEXPORT void JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1set_1JNI_1video_1buffer2
    (JNIEnv *env, jobject thiz, jobject send_vbuf_y, jobject send_vbuf_u, jobject send_vbuf_v,
    jint frame_width_px, jint frame_height_px)
{
    JNIEnv *jnienv2;
    jnienv2 = jni_getenv();
    //
    pthread_mutex_lock(&vbuffer2___mutex);
    //
    video_buffer_2 = (uint8_t *)(*jnienv2)->GetDirectBufferAddress(jnienv2, send_vbuf_y);
    jlong capacity_y = (*jnienv2)->GetDirectBufferCapacity(jnienv2, send_vbuf_y);
    video_buffer_2_y_size = (long)capacity_y;
    //
    video_buffer_2_u = (uint8_t *)(*jnienv2)->GetDirectBufferAddress(jnienv2, send_vbuf_u);
    jlong capacity_u = (*jnienv2)->GetDirectBufferCapacity(jnienv2, send_vbuf_u);
    video_buffer_2_u_size = (long)capacity_u;
    //
    video_buffer_2_v = (uint8_t *)(*jnienv2)->GetDirectBufferAddress(jnienv2, send_vbuf_v);
    jlong capacity_v = (*jnienv2)->GetDirectBufferCapacity(jnienv2, send_vbuf_v);
    video_buffer_2_v_size = (long)capacity_v;
    //
    pthread_mutex_unlock(&vbuffer2___mutex);
}

// audio_buffer is for playing audio
JNIEXPORT void JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1set_1JNI_1audio_1buffer
  (JNIEnv *env, jobject thiz, jobject send_abuf)
{
    JNIEnv *jnienv2;
    jnienv2 = jni_getenv();
    audio_buffer_pcm_1 = (uint8_t *)(*jnienv2)->GetDirectBufferAddress(jnienv2, send_abuf);
    jlong capacity = (*jnienv2)->GetDirectBufferCapacity(jnienv2, send_abuf);
    audio_buffer_pcm_1_size = (long)capacity;
}

// audio_buffer2 is for capturing audio
JNIEXPORT void JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1set_1JNI_1audio_1buffer2
  (JNIEnv *env, jobject thiz, jobject recv_abuf)
{
    JNIEnv *jnienv2;
    jnienv2 = jni_getenv();
    audio_buffer_pcm_2 = (uint8_t *)(*jnienv2)->GetDirectBufferAddress(jnienv2, recv_abuf);
    jlong capacity = (*jnienv2)->GetDirectBufferCapacity(jnienv2, recv_abuf);
    audio_buffer_pcm_2_size = (long)capacity;
}


#ifdef __cplusplus
}
#endif
