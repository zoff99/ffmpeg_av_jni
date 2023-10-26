package com.zoffcc.applications.ffmpegav;

public class AVActivity {

    private static final String TAG = "ffmpegav.AVActivity";
    static final String Version = "0.99.0";

    public static native String libavutil_version();

    public static void main(String[] args) {
        System.out.println("libavutil version: " + libavutil_version());
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

