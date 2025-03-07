#! /bin/bash

_HOME_="$(pwd)"
export _HOME_

_HOME2_=$(dirname $0)
export _HOME2_
_HOME3_=$(cd $_HOME2_;pwd)
export _HOME3_

id -a
pwd
ls -al

export _SRC_=$_HOME_/src/
export _INST_=$_HOME_/inst/


mkdir -p $_SRC_
mkdir -p $_INST_

export LD_LIBRARY_PATH=$_INST_/lib/
export PKG_CONFIG_PATH=$_INST_/lib/pkgconfig


# ----------- config ------------
ORIGPATH=$PATH
export ORIGPATH
NEWPATH=$PATH # /usr/x86_64-w64-mingw32/bin:$PATH
export NEWPATH
export PATH=$NEWPATH

# MAKEFLAGS=j$(nproc)
# export MAKEFLAGS

WGET_OPTIONS="--timeout=10"
export WGET_OPTIONS

# ----------- config ------------


## ---------------------------
pwd
ls -al
rm -fv libffmpeg_av_jni* ffmpeg_av_jni.dll
ls -al
## ---------------------------

echo "JAVADIR1------------------"
find /usr -name 'jni.h'
echo "JAVADIR1------------------"

# /usr/local/Cellar/openjdk/15.0.2/include/jni.h
# /usr/local/Cellar/openjdk/15.0.2/include/jni_md.h

# /usr/local/Cellar/openjdk/15.0.2/libexec/openjdk.jdk/Contents/Home/include/jni.h
# /usr/local/Cellar/openjdk/15.0.2/libexec/openjdk.jdk/Contents/Home/include/darwin/jni_md.h

echo "JAVADIR2------------------"
find /usr -name 'jni_md.h'
echo "JAVADIR2------------------"

#dirname $(find /usr -name 'jni.h' 2>/dev/null|grep -v 'libavcodec'|head -1) > /tmp/xx1
#dirname $(find /usr -name 'jni_md.h' 2>/dev/null|head -1) > /tmp/xx2
#export JAVADIR1=$(cat /tmp/xx1)
#export JAVADIR2=$(cat /tmp/xx2)

#export JAVADIR1="/usr/local/Cellar/openjdk/16.0.1/include"
#export JAVADIR2="/usr/local/Cellar/openjdk/16.0.1/include"

#if [ ! -e "$JAVADIR1" ]; then
#    mkdir -p "$_INST_/jinclude/"
#    cp -av /Users/travis/build/zoff99/java_toxclient_example/circle_scripts/jni_md.h "$_INST_/jinclude/"
#    cp -av /Users/travis/build/zoff99/java_toxclient_example/circle_scripts/jni.h "$_INST_/jinclude/"
#    export JAVADIR1="$_INST_/jinclude"
#    export JAVADIR2="$_INST_/jinclude"
#fi

#echo "JAVADIR1:""$JAVADIR1"
#echo "JAVADIR2:""$JAVADIR2"

mkdir -p "$_INST_/jinclude/"
cp -av "$_HOME3_"/jni_md.h "$_INST_/jinclude/"
cp -av "$_HOME3_"/jni.h "$_INST_/jinclude/"

# ------ set commit hash ------
git_hash=$(git rev-parse --verify --short=8 HEAD 2>/dev/null|tr -dc '[A-Fa-f0-9]' 2>/dev/null)
echo "XX:""$git_hash"":YY"
cat ffmpeg_av_jni.c | grep '#define FFMPEGAVJNI_GIT_COMMIT_HASH'
sed -i -e 's;#define FFMPEGAVJNI_GIT_COMMIT_HASH.*$;#define FFMPEGAVJNI_GIT_COMMIT_HASH "'$git_hash'";' ffmpeg_av_jni.c
cat ffmpeg_av_jni.c | grep '#define FFMPEGAVJNI_GIT_COMMIT_HASH'
# ------ set git commit hash ------

export CFLAGS=" -fPIC -g -O3 -std=gnu99 -I$_INST_/include/ -I$_INST_/jinclude/ -L$_INST_/lib -fstack-protector-all "

# delete first
rm -f libffmpeg_av_jni.jnilib

if [ "$1""x" == "armx" ]; then
    mac_os_min_version=""
else
    mac_os_min_version="-mmacosx-version-min=12"
fi

gcc $CFLAGS \
-Wall \
-Wno-unused-function \
-Wno-discarded-qualifiers \
-Wno-unused-const-variable \
-Wno-deprecated-declarations \
$mac_os_min_version \
-Wunguarded-availability \
-framework Foundation \
-framework CoreFoundation \
-framework CoreServices \
-framework AudioUnit \
-framework AudioToolbox \
-framework CoreAudio \
-framework AVFoundation \
-framework CoreMedia \
-framework CoreVideo \
-framework CoreGraphics \
-D_FILE_OFFSET_BITS=64 -D__USE_GNU=1 \
-I$JAVADIR1/ \
-I$JAVADIR2/ \
ffmpeg_av_jni.c \
avfoundation_list_devices.m \
$_INST_/lib/libavcodec.a \
$_INST_/lib/libavdevice.a \
$_INST_/lib/libavformat.a \
$_INST_/lib/libavfilter.a \
$_INST_/lib/libavutil.a \
$_INST_/lib/libswscale.a \
$_INST_/lib/libswresample.a \
-lpthread \
-lm \
-shared \
-o libffmpeg_av_jni.jnilib || exit 1



gcc $CFLAGS \
-Wall \
-Wno-unused-function \
-Wno-discarded-qualifiers \
-Wno-unused-const-variable \
-Wno-deprecated-declarations \
-framework Foundation \
-framework CoreFoundation \
-framework CoreServices \
-framework AudioUnit \
-framework AudioToolbox \
-framework CoreAudio \
-framework AVFoundation \
-framework CoreMedia \
-framework CoreVideo \
-framework CoreGraphics \
-D_FILE_OFFSET_BITS=64 -D__USE_GNU=1 \
test.c \
$_INST_/lib/libavcodec.a \
$_INST_/lib/libavdevice.a \
$_INST_/lib/libavformat.a \
$_INST_/lib/libavutil.a \
$_INST_/lib/libswscale.a \
$_INST_/lib/libswresample.a \
-lpthread \
-lm \
-o test_macos


ls -al libffmpeg_av_jni.jnilib || exit 1

otool -L libffmpeg_av_jni.jnilib
pwd
file libffmpeg_av_jni.jnilib

pwd
find . -name libffmpeg_av_jni.jnilib
# copy also to the mac M1 name for testing on that platform (its a bit of a hack)
cp -av libffmpeg_av_jni.jnilib libffmpeg_av_jni_arm64.jnilib
cp -av test_macos test_macos_arm

java -cp . -Djava.library.path=$(pwd) com.zoffcc.applications.ffmpegav.AVActivity
