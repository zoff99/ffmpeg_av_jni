package com.zoffcc.applications.ffmpegav;

public class AVActivity {

    private static final String TAG = "ffmpegav.AVActivity";
    static final String Version = "0.99.0";

    public static native String libavutil_version();
    public static native int init();
    public static native String[] get_video_in_devices();
    public static native int open_video_in_device(String deviceformat);
    public static native int start_video_in_capture();
    public static native int stop_video_in_capture();
    public static native int close_video_in_device();

    // buffer is for playing video
    public static native long set_JNI_video_buffer(java.nio.ByteBuffer buffer, int frame_width_px, int frame_height_px);
    // buffer2 is for capturing video
    public static native void set_JNI_video_buffer2(java.nio.ByteBuffer buffer2, int frame_width_px, int frame_height_px);
    // audio_buffer is for playing audio
    public static native void set_JNI_audio_buffer(java.nio.ByteBuffer audio_buffer);
    // audio_buffer2 is for capturing audio
    public static native void set_JNI_audio_buffer2(java.nio.ByteBuffer audio_buffer2);

    public static void callback_video_capture_frame_pts_cb_method(long width, long height, long pts)
    {
        // System.out.println("capture video frame w: " + width + " h: " + height + " pts: " + pts);
        System.out.println("capture video frame");
    }

    public static void main(String[] args) {
        System.out.println("libavutil version: " + libavutil_version());
        final int res = init();
        System.out.println("ffmpeg init: " + res);
        final String[] video_in_devices = get_video_in_devices();
        System.out.println("ffmpeg video in devices: " + video_in_devices.length);
        for (int i=0;i<video_in_devices.length;i++)
        {
            if (video_in_devices[i] != null)
            {
                System.out.println("ffmpeg video in device #"+i+": " + video_in_devices[i]);
                if (i == 1)
                {
                    final int res_vd = open_video_in_device(video_in_devices[i]);
                    System.out.println("ffmpeg open video capture device: " + res_vd);
                }
            }
        }

        final int frame_width_px1 = 640;
        final int frame_height_px1 = 480;
        final int buffer_size_in_bytes1 = ((frame_width_px1 * frame_height_px1) * 3) / 2;
        final java.nio.ByteBuffer video_buffer_1 = java.nio.ByteBuffer.allocateDirect(buffer_size_in_bytes1);
        set_JNI_video_buffer(video_buffer_1, frame_width_px1, frame_height_px1);

        final int frame_width_px2 = 640;
        final int frame_height_px2 = 480;
        final int buffer_size_in_bytes2 = ((frame_width_px2 * frame_height_px2) * 3) / 2;
        final java.nio.ByteBuffer video_buffer_2 = java.nio.ByteBuffer.allocateDirect(buffer_size_in_bytes2);
        set_JNI_video_buffer2(video_buffer_2, frame_width_px2, frame_height_px2);

        start_video_in_capture();
        try
        {
            Thread.sleep(10 * 1000);
        }
        catch(Exception e)
        {
        }
        stop_video_in_capture();

        final int res_vclose = close_video_in_device();
        System.out.println("ffmpeg open close capture device: " + res_vclose);
    }

    static
    {
        try
        {
            System.loadLibrary("ffmpeg_av_jni");
            Log.i(TAG, "successfully loaded native library");
        }
        catch (java.lang.UnsatisfiedLinkError e)
        {
            Log.i(TAG, "loadLibrary ffmpeg_av_jni failed!");
            e.printStackTrace();
            System.exit(4);
        }
    }

}

