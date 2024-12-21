#import <AVFoundation/AVFoundation.h>
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

int main(int argc, const char* argv[])
{
    NSLog(@"lsdev:start");
    NSArray *devices       = getDevicesWithMediaType(AVMediaTypeVideo);
    NSArray *devices_muxed = getDevicesWithMediaType(AVMediaTypeMuxed);
    int num_video_devices = [devices count] + [devices_muxed count];
    NSLog(@"video: %@", devices);
    NSLog(@"audio: %@", devices_muxed);


    uint32_t num_screens = 0;
#if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 1070)
    CGGetActiveDisplayList(0, NULL, &num_screens);
#endif

    int index = 0;
    NSLog(@"AVFoundation video devices:");
    for (AVCaptureDevice *device in devices) {
        const char *name = [[device localizedName] UTF8String];
        index            = [devices indexOfObject:device];
        NSLog(@"[%d] %s", index, name);
    }
    for (AVCaptureDevice *device in devices_muxed) {
        const char *name = [[device localizedName] UTF8String];
        index            = [devices count] + [devices_muxed indexOfObject:device];
        NSLog(@"[%d] %s", index, name);
    }
#if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 1070)
    if (num_screens > 0) {
        CGDirectDisplayID screens[num_screens];
        CGGetActiveDisplayList(num_screens, screens, &num_screens);
        for (int i = 0; i < num_screens; i++) {
            NSLog(@"[%d] Capture screen %d", num_video_devices + i, i);
        }
    }
#endif

    NSLog(@"AVFoundation audio devices:");
    devices = getDevicesWithMediaType(AVMediaTypeAudio);
    for (AVCaptureDevice *device in devices) {
        const char *name = [[device localizedName] UTF8String];
        int index  = [devices indexOfObject:device];
        NSLog(@"[%d] %s", index, name);
    }


    return 0;
}

