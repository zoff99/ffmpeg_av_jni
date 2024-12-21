#import <AVFoundation/AVFoundation.h>
#import <Foundation/Foundation.h>

static NSArray* getDevicesWithMediaType(AVMediaType mediaType) {
#if ((TARGET_OS_OSX && __MAC_OS_X_VERSION_MIN_REQUIRED >= 101500))
    NSMutableArray *deviceTypes = nil;
    if (mediaType == AVMediaTypeVideo) {
        deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeBuiltInWideAngleCamera]];
        #if (TARGET_OS_OSX && __MAC_OS_X_VERSION_MIN_REQUIRED >= 130000)
            [deviceTypes addObject: AVCaptureDeviceTypeDeskViewCamera];
        #endif
        #if ((TARGET_OS_OSX && __MAC_OS_X_VERSION_MIN_REQUIRED >= 140000))
            [deviceTypes addObject: AVCaptureDeviceTypeContinuityCamera];
        #endif
    } else if (mediaType == AVMediaTypeAudio) {
        #if ((TARGET_OS_OSX && __MAC_OS_X_VERSION_MIN_REQUIRED >= 140000))
            deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeMicrophone]];
        #else
            deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeBuiltInMicrophone]];
        #endif
    } else if (mediaType == AVMediaTypeMuxed) {
        #if ((TARGET_OS_OSX && __MAC_OS_X_VERSION_MIN_REQUIRED >= 140000))
            deviceTypes = [NSMutableArray arrayWithArray:@[AVCaptureDeviceTypeExternal]];
        #elif (TARGET_OS_OSX && __MAC_OS_X_VERSION_MIN_REQUIRED < 140000)
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
    NSLog(@"video: %@", devices);
    NSLog(@"audio: %@", devices_muxed);
    return 0;
}

