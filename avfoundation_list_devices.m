#import <AVFoundation/AVFoundation.h>
#include <string.h>
#import <Foundation/Foundation.h>

static NSArray* getDevicesWithMediaType(AVMediaType mediaType) {
#if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 101500)
    NSMutableArray *deviceTypes = nil;
    if (mediaType == AVMediaTypeVideo) {
        deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeBuiltInWideAngleCamera]];
        #if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 130000)
            [deviceTypes addObject: AVCaptureDeviceTypeDeskViewCamera];
        #endif
        #if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 140000)
            [deviceTypes addObject: AVCaptureDeviceTypeContinuityCamera];
        #endif
    } else if (mediaType == AVMediaTypeAudio) {
        #if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 140000)
            deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeMicrophone]];
        #else
            deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeBuiltInMicrophone]];
        #endif
    } else if (mediaType == AVMediaTypeMuxed) {
        #if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 140000)
            deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeExternal]];
        #elif (__MAC_OS_X_VERSION_MIN_REQUIRED < 140000)
            deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeExternalUnknown]];
        #else
            return nil;
        #endif
    } else {
        return nil;
    }

    AVCaptureDeviceDiscoverySession *captureDeviceDiscoverySession =
        [AVCaptureDeviceDiscoverySession
        discoverySessionWithDeviceTypes:deviceTypes
                              mediaType:mediaType
                               position:AVCaptureDevicePositionUnspecified];
    return [captureDeviceDiscoverySession devices];
#else
    return [AVCaptureDevice devicesWithMediaType:mediaType];
#endif
}

void get_device_name(int video, int num, char* str_buffer)
{
    NSArray *devices       = getDevicesWithMediaType(AVMediaTypeVideo);
    NSArray *devices_muxed = getDevicesWithMediaType(AVMediaTypeMuxed);
    int num_devices = [devices count] + [devices_muxed count];

    uint32_t num_screens = 0;
#if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 1070)
    CGGetActiveDisplayList(0, NULL, &num_screens);
#endif

    int index = 0;
    // NSLog(@"AVFoundation video devices:");
    for (AVCaptureDevice *device in devices) {
        const char *name = [[device localizedName] UTF8String];
        index            = [devices indexOfObject:device];
        if ((video == 1) && (index == num)) {
            NSLog(@"VV1:[%d] %s", index, name);
        }
    }
    for (AVCaptureDevice *device in devices_muxed) {
        const char *name = [[device localizedName] UTF8String];
        index            = [devices count] + [devices_muxed indexOfObject:device];
        if ((video == 1) && (index == num)) {
            NSLog(@"VV2:[%d] %s", index, name);
        }
    }
#if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 1070)
    if (num_screens > 0) {
        CGDirectDisplayID screens[num_screens];
        CGGetActiveDisplayList(num_screens, screens, &num_screens);
        for (int i = 0; i < num_screens; i++) {
            if ((video == 1) && ((num_devices + i) == num)) {
                NSLog(@"VV3:[%d] Capture screen %d", num_devices + i, i);
            }
        }
    }
#endif

    // NSLog(@"AVFoundation audio devices:");
    devices = getDevicesWithMediaType(AVMediaTypeAudio);
    for (AVCaptureDevice *device in devices) {
        const char *name = [[device localizedName] UTF8String];
        int index  = [devices indexOfObject:device];
        if ((video == 0) && (index == num)) {
            NSLog(@"AA1:[%d] %s", index, name);
        }
    }


}

int main(int argc, const char* argv[])
{
    char buf[10000];
    for (int i=0;i<20;i++) {
        NSLog(@"V:%d", i);
        memset(buf, 0, 10000);
        get_device_name(1, i, buf);
    }

    for (int i=0;i<20;i++) {
        NSLog(@"A:%d", i);
        memset(buf, 0, 10000);
        get_device_name(0, i, buf);
    }
}