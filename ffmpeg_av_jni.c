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

#ifdef __APPLE__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifndef OS_WIN32
#include <sys/time.h>
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
static uint64_t ffav_current_time_monotonic_default()
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

static size_t ffav_xnet_pack_u16(uint8_t *bytes, uint16_t v)
{
    bytes[0] = (v >> 8) & 0xff;
    bytes[1] = v & 0xff;
    return sizeof(v);
}

static size_t ffav_xnet_pack_u32(uint8_t *bytes, uint32_t v)
{
    uint8_t *p = bytes;
    p += ffav_xnet_pack_u16(p, (v >> 16) & 0xffff);
    p += ffav_xnet_pack_u16(p, v & 0xffff);
    return p - bytes;
}

static size_t ffav_xnet_unpack_u16(const uint8_t *bytes, uint16_t *v)
{
    uint8_t hi = bytes[0];
    uint8_t lo = bytes[1];
    *v = ((uint16_t)hi << 8) | lo;
    return sizeof(*v);
}

static size_t ffav_xnet_unpack_u32(const uint8_t *bytes, uint32_t *v)
{
    const uint8_t *p = bytes;
    uint16_t hi;
    uint16_t lo;
    p += ffav_xnet_unpack_u16(p, &hi);
    p += ffav_xnet_unpack_u16(p, &lo);
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

// --------- AV VARS ---------
// --------- AV VARS ---------
// --------- AV VARS ---------
AVInputFormat *inputFormat = NULL;
AVFormatContext *formatContext = NULL;
AVDictionary *options = NULL;
int videoStreamIndex = -1;
bool global_video_in_capture_running = false;
static pthread_t ffmpeg_thread_video_in_capture;

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
    videoStreamIndex = -1;
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
    while (global_video_in_capture_running == true)
    {
        yieldcpu(1000);
    }

    printf("ffmpeg Video Capture Thread:Clean thread exit!\n");
    return NULL;
}

JNIEXPORT jstring JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_libavutil_1version(JNIEnv *env, jobject thiz)
{
    char libavutil_version_str[2000];
    CLEAR(libavutil_version_str);
    snprintf(libavutil_version_str, 1999, "%d.%d.%d", (int)LIBAVUTIL_VERSION_MAJOR, (int)LIBAVUTIL_VERSION_MINOR, (int)LIBAVUTIL_VERSION_MICRO);
    return (*env)->NewStringUTF(env, libavutil_version_str);
}

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_init(JNIEnv *env, jobject thiz)
{
#if (LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58,9,100))
    av_register_all();
#endif
    // avformat_network_init();
    avdevice_register_all();

    // HINT: add error handling
    return 0;
}

JNIEXPORT jobjectArray JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_get_1video_1in_1devices(JNIEnv *env, jobject thiz)
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

JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_open_1video_1in_1device(JNIEnv *env, jobject thiz, jstring deviceformat)
{
    const char *deviceformat_cstr = (*env)->GetStringUTFChars(env, deviceformat, NULL);

    inputFormat = av_find_input_format(deviceformat_cstr);
    (*env)->ReleaseStringUTFChars(env, deviceformat, deviceformat_cstr);
    if (!inputFormat) {
        fprintf(stderr, "Could not find input format\n");
        reset_video_in_values();
        return -1;
    }

    if (avformat_open_input(&formatContext, ":0.0", inputFormat, &options) < 0) {
        fprintf(stderr, "Could not open input\n");
        reset_video_in_values();
        return -1;
    }

    if (avformat_find_stream_info(formatContext, NULL) < 0) {
        fprintf(stderr, "Could not find stream info\n");
        avformat_close_input(&formatContext);
        av_dict_free(&options);
        reset_video_in_values();
        return -1;
    }

    for (int i = 0; i < formatContext->nb_streams; i++) {
        if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoStreamIndex = i;
            break;
        }
    }

    if (videoStreamIndex == -1) {
        fprintf(stderr, "Could not find video stream\n");
        avformat_close_input(&formatContext);
        av_dict_free(&options);
        reset_video_in_values();
        return -1;
    }

    return 0;
}

JNIEXPORT jint JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_start_1video_1in_1capture(JNIEnv *env, jobject thiz)
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

JNIEXPORT jint JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_stop_1video_1in_1capture(JNIEnv *env, jobject thiz)
{
    global_video_in_capture_running = false;
    // wait for capture thread to finish
    pthread_join(ffmpeg_thread_video_in_capture, NULL);
    return 0;
}


JNIEXPORT jint JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_close_1video_1in_1device(JNIEnv *env, jobject thiz)
{
    avformat_close_input(&formatContext);
    av_dict_free(&options);
    return 0;
}


// buffer is for incoming video (call)
JNIEXPORT void JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_set_1JNI_1video_1buffer
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

// buffer2 is for sending video (call)
JNIEXPORT void JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_set_1JNI_1video_1buffer2
  (JNIEnv *env, jobject thiz, jobject send_vbuf, jint frame_width_px, jint frame_height_px)
{
    JNIEnv *jnienv2;
    jnienv2 = jni_getenv();
    video_buffer_2 = (uint8_t *)(*jnienv2)->GetDirectBufferAddress(jnienv2, send_vbuf);
    jlong capacity = (*jnienv2)->GetDirectBufferCapacity(jnienv2, send_vbuf);
    video_buffer_2_size = (long)capacity;
}

// audio_buffer is for sending audio (group and call)
JNIEXPORT void JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_set_1JNI_1audio_1buffer
  (JNIEnv *env, jobject thiz, jobject send_abuf)
{
    JNIEnv *jnienv2;
    jnienv2 = jni_getenv();
    audio_buffer_pcm_1 = (uint8_t *)(*jnienv2)->GetDirectBufferAddress(jnienv2, send_abuf);
    jlong capacity = (*jnienv2)->GetDirectBufferCapacity(jnienv2, send_abuf);
    audio_buffer_pcm_1_size = (long)capacity;
}

// audio_buffer2 is for incoming audio (group and call)
JNIEXPORT void JNICALL Java_com_zoffcc_applications_ffmpegav_AVActivity_set_1JNI_1audio_1buffer2
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
