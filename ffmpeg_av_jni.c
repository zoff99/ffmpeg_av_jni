#define _GNU_SOURCE

#include <ctype.h>
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

#include <libavutil/avutil.h>

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


// ----- JNI stuff -----
JNIEnv *jnienv;
JavaVM *cachedJVM = NULL;


// gives a counter value that increaes every millisecond
static uint64_t current_time_monotonic_default()
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

size_t xnet_pack_u16(uint8_t *bytes, uint16_t v)
{
    bytes[0] = (v >> 8) & 0xff;
    bytes[1] = v & 0xff;
    return sizeof(v);
}

size_t xnet_pack_u32(uint8_t *bytes, uint32_t v)
{
    uint8_t *p = bytes;
    p += xnet_pack_u16(p, (v >> 16) & 0xffff);
    p += xnet_pack_u16(p, v & 0xffff);
    return p - bytes;
}

size_t xnet_unpack_u16(const uint8_t *bytes, uint16_t *v)
{
    uint8_t hi = bytes[0];
    uint8_t lo = bytes[1];
    *v = ((uint16_t)hi << 8) | lo;
    return sizeof(*v);
}

size_t xnet_unpack_u32(const uint8_t *bytes, uint32_t *v)
{
    const uint8_t *p = bytes;
    uint16_t hi;
    uint16_t lo;
    p += xnet_unpack_u16(p, &hi);
    p += xnet_unpack_u16(p, &lo);
    *v = ((uint32_t)hi << 16) | lo;
    return p - bytes;
}



// ------------- JNI -------------
// ------------- JNI -------------
// ------------- JNI -------------

JNIEXPORT jstring JNICALL
Java_com_zoffcc_applications_ffmpegav_AVActivity_libavutil_1version(JNIEnv *env, jobject thiz)
{
    char libavutil_version_str[2000];
    CLEAR(libavutil_version_str);
    snprintf(libavutil_version_str, 1999, "%d.%d.%d", (int)LIBAVUTIL_VERSION_MAJOR, (int)LIBAVUTIL_VERSION_MINOR, (int)LIBAVUTIL_VERSION_MICRO);
    return (*env)->NewStringUTF(env, libavutil_version_str);
}



