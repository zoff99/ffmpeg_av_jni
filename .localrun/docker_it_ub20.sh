#! /bin/bash

_HOME2_=$(dirname $0)
export _HOME2_
_HOME_=$(cd $_HOME2_;pwd)
export _HOME_

echo $_HOME_
cd $_HOME_

if [ "$1""x" == "buildx" ]; then
    docker build -f Dockerfile_ub20 -t ffmpeg_av_jni_ready_ub20_001 .
    exit 0
fi

build_for='
ubuntu:20.04
'

for system_to_build_for in $build_for ; do

    system_to_build_for_orig="$system_to_build_for"
    system_to_build_for=$(echo "$system_to_build_for_orig" 2>/dev/null|tr ':' '_' 2>/dev/null)

    cd $_HOME_/
    mkdir -p $_HOME_/"$system_to_build_for"/

    mkdir -p $_HOME_/"$system_to_build_for"/artefacts
    mkdir -p $_HOME_/"$system_to_build_for"/script
    mkdir -p $_HOME_/"$system_to_build_for"/workspace/build/

    ls -al $_HOME_/"$system_to_build_for"/

    rsync -a ../ffmpeg_av_jni.c --exclude=.localrun $_HOME_/"$system_to_build_for"/workspace/build/
    chmod a+rwx -R $_HOME_/"$system_to_build_for"/workspace/build >/dev/null 2>/dev/null

    echo '#! /bin/bash

cd /workspace/build/

_HOME_="$(pwd)"
export _HOME_

cd "$_HOME_"

export _SRC_=$_HOME_/src/
export _INST_=$_HOME_/inst/

mkdir -p $_SRC_
mkdir -p $_INST_

export LD_LIBRARY_PATH=$_INST_/lib/
export PKG_CONFIG_PATH=$_INST_/lib/pkgconfig

echo "*** compile ***"

cd /workspace2/build/
cd ./FFmpeg-n6.0/

# ./configure --help
# ./configure --list-outdevs

  ./configure  \
              --enable-gpl \
              --prefix="$_INST_" \
              --disable-asm \
              --enable-pic \
              --disable-swscale \
              --disable-network \
              --disable-everything \
              --disable-debug \
              --disable-shared \
              --disable-programs \
              --disable-protocols \
              --disable-doc \
              --disable-sdl2 \
              --disable-avfilter \
              --disable-filters \
              --disable-iconv \
              --disable-network \
              --disable-postproc \
              --disable-swresample \
              --disable-swscale-alpha \
              --disable-dct \
              --disable-dwt \
              --disable-lsp \
              --disable-mdct \
              --disable-rdft \
              --disable-fft \
              --disable-faan \
              --disable-vaapi \
              --disable-vdpau \
              --disable-zlib \
              --disable-bzlib \
              --disable-lzma \
              --disable-encoders \
              --disable-decoders \
              --disable-demuxers \
              --disable-parsers \
              --disable-bsfs \
              --enable-outdev=alsa \
              --enable-outdev=pulse \
              --enable-indev=alsa \
              --enable-indev=xcbgrab \
              --enable-indev=pulse \
              --enable-indev=v4l2 \
              --enable-libpulse \
              --enable-libv4l2 \
              || exit 1

make -j$(nproc) || exit 1
make install || exit 1

ls -al "$_INST_"/lib/

# libavcodec.a
# libavdevice.a
# libavformat.a
# libavutil.a

cd /workspace/build/
gcc --version

ls -al


echo "JAVADIR1------------------"
find /usr -name "jni.h"
echo "JAVADIR1------------------"

echo "JAVADIR2------------------"
find /usr -name "jni_md.h"
echo "JAVADIR2------------------"


dirname $(find /usr -name "jni.h" 2>/dev/null|grep -v "libavcodec"|head -1) > /tmp/xx1
dirname $(find /usr -name "jni_md.h" 2>/dev/null|head -1) > /tmp/xx2
export JAVADIR1=$(cat /tmp/xx1)
export JAVADIR2=$(cat /tmp/xx2)
echo "JAVADIR1:""$JAVADIR1"
echo "JAVADIR2:""$JAVADIR2"

export CFLAGS=" -fPIC -std=gnu99 -I$_INST_/include/ -L$_INST_/lib -fstack-protector-all "


gcc $CFLAGS \
-Wall \
-DJAVA_LINUX \
$C_FLAGS \
-D_FILE_OFFSET_BITS=64 -D__USE_GNU=1 \
-I$JAVADIR1/ \
-I$JAVADIR2/ \
ffmpeg_av_jni.c \
$_INST_/lib/libavcodec.a \
$_INST_/lib/libavdevice.a \
$_INST_/lib/libavformat.a \
$_INST_/lib/libavutil.a \
-lpthread \
-lm \
-shared \
-Wl,-soname,libffmpeg_av_jni.so -o libffmpeg_av_jni.so || exit 1


#Linux:
# g++ -shared -fPIC -o libnative.so com_baeldung_jni_HelloWorldJNI.o -lc
# Windows version:
# g++ -shared -o native.dll com_baeldung_jni_HelloWorldJNI.o -Wl,--add-stdcall-alias
# MacOS version:
# g++ -dynamiclib -o libnative.dylib com_baeldung_jni_HelloWorldJNI.o -lc




ls -al libffmpeg_av_jni.so || exit 1
pwd
file libffmpeg_av_jni.so
cp -v libffmpeg_av_jni.so /artefacts/ || exit 1

chmod a+rw /artefacts/*

' > $_HOME_/"$system_to_build_for"/script/run.sh

    mkdir -p $_HOME_/"$system_to_build_for"/workspace/build/c-toxcore/

    docker run -ti --rm \
      -v $_HOME_/"$system_to_build_for"/artefacts:/artefacts \
      -v $_HOME_/"$system_to_build_for"/script:/script \
      -v $_HOME_/"$system_to_build_for"/workspace:/workspace \
      --net=host \
     "ffmpeg_av_jni_ready_ub20_001" \
     /bin/sh -c "apk add bash >/dev/null 2>/dev/null; /bin/bash /script/run.sh"
     if [ $? -ne 0 ]; then
        echo "** ERROR **:$system_to_build_for_orig"
        exit 1
     else
        echo "--SUCCESS--:$system_to_build_for_orig"
     fi

done


