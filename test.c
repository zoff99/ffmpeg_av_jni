#include <stdio.h>
#include <libavdevice/avdevice.h>
#include <libavutil/dict.h>

/* gcc -g test.c $(pkg-config --libs --cflags libavdevice libavcodec libavformat libavutil) -o test */

void sources(const char* devicename)
{
    AVInputFormat *inputFormat = av_find_input_format(devicename);
    AVDeviceInfoList *deviceInfoList = NULL;
    AVDeviceInfo *deviceInfo = NULL;
    int i;
    printf("    %p\n", inputFormat);
    printf("    %s\n", inputFormat->name);
    printf("    %d\n", inputFormat->flags);
    int s = avdevice_list_input_sources(inputFormat, NULL, NULL, &deviceInfoList);
    printf("    %d\n", s);
	if (s > 0)
    {
        for (i = 0; i < deviceInfoList->nb_devices; i++) {
            deviceInfo = deviceInfoList->devices[i];
            printf("    ==> Input Device        #%d: %s\n", i, deviceInfo->device_name);
            if (deviceInfo->device_description) {
                printf("    ==> Input Device descr. #%d: %s\n", i, deviceInfo->device_description);
            }
        }
    }
    avdevice_free_list_devices(&deviceInfoList);
}

void get_video_in_devices()
{
    printf("\n");
    printf("\n");
    printf("Video In Devices:\n");
    printf("=================\n");
    const uint32_t max_devices = 64;
    AVInputFormat *inputFormat_search = av_input_video_device_next(NULL);
    uint32_t in_device_count = 0;
    while (inputFormat_search != NULL) {
        printf("==> Input Device Name: %s long name: %s\n", inputFormat_search->name, inputFormat_search->long_name);
        in_device_count++;
        sources(inputFormat_search->name);
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
}

void get_audio_in_devices()
{
    printf("\n");
    printf("\n");
    printf("Audio In Devices:\n");
    printf("=================\n");
    const uint32_t max_devices = 64;
    AVInputFormat *inputFormat_search = av_input_audio_device_next(NULL);
    uint32_t in_device_count = 0;
    while (inputFormat_search != NULL) {
        printf("==> Input Device Name: %s long name: %s\n", inputFormat_search->name, inputFormat_search->long_name);
        in_device_count++;
        sources(inputFormat_search->name);
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
}

void list_all()
{
    get_video_in_devices();
    get_audio_in_devices();
}

int main(int argc, char *argv[]) {
    avdevice_register_all();

    list_all();

    return 0;
}
