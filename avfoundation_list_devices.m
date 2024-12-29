#import <AVFoundation/AVFoundation.h>
#import <Foundation/Foundation.h>
#include <string.h>
#include <stdio.h>

#define GETMIN(a,b) ((a) < (b) ? (a) : (b))


static void getVideoDevicesPermissionObjc() {
// HINT: copyied from qtox source code
#if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 101400)
    const AVAuthorizationStatus authStatus =
        [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    if (authStatus != AVAuthorizationStatusDenied && authStatus != AVAuthorizationStatusNotDetermined) {
        printf("We already have access to the camera\n");
    } else {
        printf("We don't have access to the camera yet; asking user for permission\n");
        __block BOOL isGranted = false;
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
                                completionHandler:^(BOOL granted) {
                                isGranted = granted;
                                }];
        if (isGranted) {
            printf("We now have access to the camera\n");
        } else {
            printf("User did not grant us permission to access the camera\n");
        }
    }
#endif
}

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

void getDevicesPermission(int want_video)
{
    getVideoDevicesPermissionObjc();
}

void get_device_name(int video, int num, char* str_buffer, int str_buffer_len)
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
            // NSLog(@"VV1:[%d] %s", index, name);
            memcpy(str_buffer, name, GETMIN(str_buffer_len, strlen(name)));
        }
    }
    for (AVCaptureDevice *device in devices_muxed) {
        const char *name = [[device localizedName] UTF8String];
        index            = [devices count] + [devices_muxed indexOfObject:device];
        if ((video == 1) && (index == num)) {
            // NSLog(@"VV2:[%d] %s", index, name);
            memcpy(str_buffer, name, GETMIN(str_buffer_len, strlen(name)));
        }
    }
#if (__MAC_OS_X_VERSION_MIN_REQUIRED >= 1070)
    if (num_screens > 0) {
        CGDirectDisplayID screens[num_screens];
        CGGetActiveDisplayList(num_screens, screens, &num_screens);
        for (int i = 0; i < num_screens; i++) {
            if ((video == 1) && ((num_devices + i) == num)) {
                // NSLog(@"VV3:[%d] Capture screen %d", num_devices + i, i);
                const int tmpstr_len = 100;
                char tmpstr[tmpstr_len];
                memset(tmpstr, 0, tmpstr_len);
                snprintf(tmpstr, tmpstr_len - 1, "Capture screen %d", (num_devices + i));
                memcpy(str_buffer, tmpstr, GETMIN(str_buffer_len, strlen(tmpstr)));
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
            // NSLog(@"AA1:[%d] %s", index, name);
            memcpy(str_buffer, name, GETMIN(str_buffer_len, strlen(name)));
        }
    }


}

/*
int main(int argc, const char* argv[])
{
    const int buf_len = 200;
    char buf[buf_len];
    for (int i=0;i<20;i++) {
        // NSLog(@"V:%d", i);
        memset(buf, 0, buf_len);
        get_device_name(1, i, buf, (buf_len - 1));
        if (buf[0]) {
            printf("VC:%d:%s\n", i, buf);
        }
    }

    for (int i=0;i<20;i++) {
        // NSLog(@"A:%d", i);
        memset(buf, 0, buf_len);
        get_device_name(0, i, buf,(buf_len - 1));
        if (buf[0]) {
            printf("VA:%d:%s\n", i, buf);
        }
    }
}
*/
