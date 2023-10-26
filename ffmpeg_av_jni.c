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
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/time.h>
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
#define VERSION_PATCH 0
static const char global_version_string[] = "0.99.0";
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

// --------- AV VARS ---------
// --------- AV VARS ---------
// --------- AV VARS ---------
AVInputFormat *inputFormat = NULL;
AVFormatContext *formatContext = NULL;
AVCodecContext *global_video_codec_ctx = NULL;
AVDictionary *options = NULL;
int video_stream_index = -1;
AVCodec *video_codec = NULL;
bool global_video_in_capture_running = false;
static pthread_t ffmpeg_thread_video_in_capture;
AVRational time_base_video = (AVRational) {0, 0};

#define DEFAULT_SCREEN_CAPTURE_FPS "20" // 20 fps desktop screen capture
char* global_desktop_display_num_str = NULL;
int sws_scale_algo = SWS_FAST_BILINEAR; // SWS_FAST_BILINEAR SWS_BILINEAR SWS_BICUBIC SWS_SINC SWS_LANCZOS
int output_width = 640;
int output_height = 480;

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
    inputFormat = NULL;
    formatContext = NULL;
    options = NULL;
    video_stream_index = -1;
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

static void *ffmpeg_thread_video_in_capture_func(void *data)
{
    AVPacket packet;
    AVFrame *frame = NULL;
    int ret = 0;

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

    uint8_t *dst_yuv_buffer[3];
    dst_yuv_buffer[0] = yuv_buffer;
    dst_yuv_buffer[1] = yuv_buffer + (output_width * output_height);
    dst_yuv_buffer[2] = dst_yuv_buffer[1] + ((output_width * output_height) / 4);

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

    while (global_video_in_capture_running == true)
    {
        if (av_read_frame(formatContext, &packet) >= 0)
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
                    // Convert the video frame to YUV
                    int planes_stride[3];
                    planes_stride[0] = av_image_get_linesize(AV_PIX_FMT_YUV420P, output_width, 0);
                    planes_stride[1] = av_image_get_linesize(AV_PIX_FMT_YUV420P, output_width, 1);
                    planes_stride[2] = av_image_get_linesize(AV_PIX_FMT_YUV420P, output_width, 2);
                    fprintf(stderr, "VideoFrame:strides:%d %d %d\n",planes_stride[0],planes_stride[1],planes_stride[2]);

                    sws_scale(scaler_ctx, (const uint8_t * const*)frame->data, frame->linesize, 0, global_video_codec_ctx->height,
                            dst_yuv_buffer, planes_stride);

                    if (
                        (video_buffer_2_y_size >= (planes_stride[0] * output_height)) &&
                        (video_buffer_2_u_size >= (planes_stride[1] * (output_height / 2))) &&
                        (video_buffer_2_v_size >= (planes_stride[2] * (output_height / 2)))
                        )
                    {

                        memcpy(video_buffer_2, dst_yuv_buffer[0], planes_stride[0] * output_height);
                        memcpy(video_buffer_2_u, dst_yuv_buffer[1], planes_stride[1] * (output_height / 2));
                        memcpy(video_buffer_2_v, dst_yuv_buffer[2], planes_stride[2] * (output_height / 2));

                        if (jnienv2 != NULL) {
                        (*jnienv2)->CallStaticVoidMethod(jnienv2, AVActivity,
                             callback_video_capture_frame_pts_cb_method,
                             (jlong)output_width,
                             (jlong)output_height,
                             (jlong)0
                            );
                        }
                        else
                        {
                            fprintf(stderr, "could not attach thread to JVM\n");
                        }
                    }
                    else
                    {
                        fprintf(stderr, "video buffer to small for video frame data\n");
                    }
                }
            }
            av_packet_unref(&packet);
        }
        // yieldcpu(100);
    }

    av_frame_free(&frame);
    av_free(yuv_buffer);
    sws_freeContext(scaler_ctx);

    if (cachedJVM)
    {
        (*cachedJVM)->DetachCurrentThread(cachedJVM);
    }

    printf("ffmpeg Video Capture Thread:Clean thread exit!\n");
    return NULL;
}

JNIEXPORT jstring JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1libavutil_1version(JNIEnv *env, jobject thiz)
{
    char libavutil_version_str[2000];
    CLEAR(libavutil_version_str);
    snprintf(libavutil_version_str, 1999, "%d.%d.%d", (int)LIBAVUTIL_VERSION_MAJOR, (int)LIBAVUTIL_VERSION_MINOR, (int)LIBAVUTIL_VERSION_MICRO);
    return (*env)->NewStringUTF(env, libavutil_version_str);
}

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1init(JNIEnv *env, jobject thiz)
{
#if (LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58,9,100))
    av_register_all();
#endif
    // avformat_network_init();
    avdevice_register_all();

    jclass cls_local = (*env)->GetObjectClass(env, thiz);
// android
//    AVActivity = (*env)->NewGlobalRef(env, cls_local);
// JAVA_LINUX
    java_find_class_global("com/zoffcc/applications/ffmpegav/AVActivity", &AVActivity);

    printf("cls_local=%p\n", cls_local);
    printf("AVActivity=%p\n", AVActivity);

    callback_video_capture_frame_pts_cb_method = (*env)->GetStaticMethodID(env, AVActivity,
            "ffmpegav_callback_video_capture_frame_pts_cb_method", "(JJJ)V");

    printf("ffmpegav_callback_video_capture_frame_pts_cb_method=%p\n", callback_video_capture_frame_pts_cb_method);

    // HINT: add error handling
    return 0;
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

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1open_1video_1in_1device(JNIEnv *env, jobject thiz, jstring deviceformat,
    jint wanted_width, jint wanted_height, jstring x11_display_num, jint fps)
{
    if (deviceformat == NULL)
    {
        return -1;
    }
    const char *deviceformat_cstr = (*env)->GetStringUTFChars(env, deviceformat, NULL);
    if (deviceformat_cstr == NULL)
    {
        return -1;
    }
    printf("wanted_video_in_device=%s\n", deviceformat_cstr);
    fprintf(stderr, "wanted_video_capture_resolution: %dx%d\n", wanted_width, wanted_height);

    output_width = wanted_width;
    output_height = wanted_height;

    inputFormat = av_find_input_format(deviceformat_cstr);
    if (!inputFormat) {
        fprintf(stderr, "Could not find input format\n");
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
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

        AVInputFormat *ifmt = av_find_input_format("x11grab");

        const int desktop_display_cap_str_len = 1000;
        char desktop_display_cap_str[desktop_display_cap_str_len];
        memset(desktop_display_cap_str, 0, desktop_display_cap_str_len);
        if (global_desktop_display_num_str == NULL)
        {
            global_desktop_display_num_str = ":0.0";
        }
        snprintf(desktop_display_cap_str, desktop_display_cap_str_len, "%s+0,0", global_desktop_display_num_str);
        fprintf(stderr, "Display capture_string: %s\n", desktop_display_cap_str);
#endif

        // example: grab at position 10,20 ":0.0+10,20"
        if (avformat_open_input(&formatContext, desktop_display_cap_str, ifmt, &options) != 0)
        {
            fprintf(stderr, "Could not open desktop as video input stream.\n");
            reset_video_in_values();
            (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
            return -1;
        }
    }
    else if (avformat_open_input(&formatContext, ":0.0", inputFormat, &options) < 0) {
        fprintf(stderr, "Could not open input\n");
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        return -1;
    }

    if (avformat_find_stream_info(formatContext, NULL) < 0) {
        fprintf(stderr, "Could not find stream info\n");
        avformat_close_input(&formatContext);
        av_dict_free(&options);
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        return -1;
    }

    for (int i = 0; i < formatContext->nb_streams; i++) {
        AVCodecParameters *codec_params = formatContext->streams[i]->codecpar;
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
            time_base_video = formatContext->streams[i]->time_base;
            avcodec_free_context(&video_codec_ctx);
            global_video_codec_ctx = avcodec_alloc_context3(codec);
            avcodec_parameters_to_context(global_video_codec_ctx, codec_params);
            avcodec_open2(global_video_codec_ctx, codec, NULL);            
            break;
        }
    }

    if (video_stream_index < 0) {
        fprintf(stderr, "Could not find video stream\n");
        avformat_close_input(&formatContext);
        av_dict_free(&options);
        reset_video_in_values();
        (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
        return -1;
    }

    (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
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
        pthread_setname_np(ffmpeg_thread_video_in_capture, "t_ff_vic");
        printf("ffmpeg Video Capture Thread successfully created\n");
        return 0;
    }
    return -1;
}

JNIEXPORT jint JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1stop_1video_1in_1capture(JNIEnv *env, jobject thiz)
{
    global_video_in_capture_running = false;
    // wait for capture thread to finish
    pthread_join(ffmpeg_thread_video_in_capture, NULL);
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_ffmpegav_1close_1video_1in_1device(JNIEnv *env, jobject thiz)
{
    avcodec_free_context(&global_video_codec_ctx);
    avformat_close_input(&formatContext);
    av_dict_free(&options);
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
