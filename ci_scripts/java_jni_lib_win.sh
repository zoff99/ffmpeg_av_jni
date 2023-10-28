#! /bin/bash

_HOME_="$(pwd)"
export _HOME_

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
NEWPATH=/usr/x86_64-w64-mingw32/bin:$PATH
export NEWPATH
export PATH=$NEWPATH

MAKEFLAGS=j$(nproc)
export MAKEFLAGS

WGET_OPTIONS="--timeout=10"
export WGET_OPTIONS

ARCH="x86_64"
export ARCH
# ----------- config ------------


## ---------------------------
pwd
ls -al

## ---------------------------

echo "JAVADIR1------------------"
find /usr -name 'jni.h'
echo "JAVADIR1------------------"

echo "JAVADIR2------------------"
find /usr -name 'jni_md.h'
echo "JAVADIR2------------------"

dirname $(find /usr -name 'jni.h' 2>/dev/null|grep -v 'libavcodec'|head -1) > /tmp/xx1
dirname $(find /usr -name 'jni_md.h' 2>/dev/null|head -1) > /tmp/xx2
export JAVADIR1=$(cat /tmp/xx1)
export JAVADIR2=$(cat /tmp/xx2)
echo "JAVADIR1:""$JAVADIR1"
echo "JAVADIR2:""$JAVADIR2"

export CFLAGS=" -fPIC -std=gnu99 -I$_INST_/include/ -L$_INST_/lib -fstack-protector-all "

x86_64-w64-mingw32-gcc $CFLAGS \
-Wall -D_JNI_IMPLEMENTATION_ -Wl,-kill-at \
-DJAVA_LINUX \
$C_FLAGS $CXX_FLAGS $LD_FLAGS \
-D_FILE_OFFSET_BITS=64 -D__USE_GNU=1 \
-I$JAVADIR1/ \
-I$JAVADIR2/ \
ffmpeg_av_jni.c \
$_INST_/lib/libavdevice.a \
$_INST_/lib/libswscale.a \
$_INST_/lib/libavformat.a \
$_INST_/lib/libavcodec.a \
$_INST_/lib/libswresample.a \
$_INST_/lib/libavutil.a \
-Wl,-Bstatic -lws2_32 \
-l:libiphlpapi.a \
-l:libshlwapi.a \
-l:libole32.a \
-l:libstrmiids.a \
-l:liboleaut32.a \
-l:libuuid.a \
-Wl,-Bstatic -lbcrypt \
-shared \
-lpthread \
-lm \
-o libffmpeg_av_jni.dll || exit 1


ls -al libffmpeg_av_jni.dll || exit 1
pwd
file libffmpeg_av_jni.dll

# -------------- now compile the JNI lib ----------------------
