#! /bin/bash

_HOME2_=$(dirname $0)
export _HOME2_
_HOME_=$(cd $_HOME2_;pwd)
export _HOME_

echo $_HOME_
cd $_HOME_

if [ "$1""x" == "buildx" ]; then
    docker build -f Dockerfile_ub22 -t ffmpeg_av_jni_ready_ub22_win_001 .
    exit 0
fi

build_for='
windows_ub22
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
    rsync -a ../test.c --exclude=.localrun $_HOME_/"$system_to_build_for"/workspace/build/
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

export _INST2_="/workspace2/build/inst/"

export LD_LIBRARY_PATH="$_INST2_"/lib/
export PKG_CONFIG_PATH="$_INST2_"/lib/pkgconfig

echo "*** compile ***"

ls -al "$_INST2_"/lib/

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


export CFLAGS=" -fPIC -std=gnu99 -I$_INST2_/include/ -L$_INST2_/lib -fstack-protector-all -D_FORTIFY_SOURCE=2 "

x86_64-w64-mingw32-gcc $CFLAGS \
-Wno-format-extra-args \
-Wno-deprecated-declarations \
-Wno-format \
-Wno-unused-function \
-Wno-discarded-qualifiers \
-Wno-unused-const-variable \
-Wall -D_JNI_IMPLEMENTATION_ -Wl,-kill-at \
-DJAVA_LINUX \
$C_FLAGS \
-D_FILE_OFFSET_BITS=64 -D__USE_GNU=1 \
-I$JAVADIR1/ \
-I$JAVADIR2/ \
ffmpeg_av_jni.c \
$_INST2_/lib/libavdevice.a \
$_INST2_/lib/libswscale.a \
$_INST2_/lib/libavformat.a \
$_INST2_/lib/libavfilter.a \
$_INST2_/lib/libavcodec.a \
$_INST2_/lib/libswresample.a \
$_INST2_/lib/libavutil.a \
-Wl,-Bstatic -lws2_32 \
-l:libiphlpapi.a \
-l:libshlwapi.a \
-l:libole32.a \
-l:libstrmiids.a \
-l:liboleaut32.a \
-l:libuuid.a \
-l:libgdi32.a \
-l:libavicap32.a \
-l:libvfw32.a \
-l:libpsapi.a \
-l:libshell32.a \
-Wl,-Bstatic -lbcrypt \
-shared \
-lpthread \
-lm \
-o ffmpeg_av_jni.dll || exit 1


x86_64-w64-mingw32-gcc $CFLAGS \
-Wno-format-extra-args \
-Wno-deprecated-declarations \
-Wno-format \
-Wno-unused-function \
-Wno-discarded-qualifiers \
-Wno-unused-const-variable \
-Wall \
-DJAVA_LINUX \
$C_FLAGS \
-D_FILE_OFFSET_BITS=64 -D__USE_GNU=1 \
-I$JAVADIR1/ \
-I$JAVADIR2/ \
test.c \
$_INST2_/lib/libavdevice.a \
$_INST2_/lib/libswscale.a \
$_INST2_/lib/libavformat.a \
$_INST2_/lib/libavfilter.a \
$_INST2_/lib/libavcodec.a \
$_INST2_/lib/libswresample.a \
$_INST2_/lib/libavutil.a \
-Wl,-Bstatic -lws2_32 \
-l:libiphlpapi.a \
-l:libshlwapi.a \
-l:libole32.a \
-l:libstrmiids.a \
-l:liboleaut32.a \
-l:libuuid.a \
-l:libgdi32.a \
-l:libavicap32.a \
-l:libvfw32.a \
-l:libpsapi.a \
-l:libshell32.a \
-Wl,-Bstatic -lbcrypt \
-lpthread \
-lm \
-o test.exe
ls -al test.exe
cp -av test.exe /artefacts/

cp -av /workspace2/build/inst/bin/ffmpeg.exe /artefacts/





ls -al ffmpeg_av_jni.dll || exit 1
pwd
file ffmpeg_av_jni.dll

cp -av ffmpeg_av_jni.dll /artefacts/

chmod a+rw /artefacts/*

' > $_HOME_/"$system_to_build_for"/script/run.sh

    mkdir -p $_HOME_/"$system_to_build_for"/workspace/build/c-toxcore/

    docker run -ti --rm \
      -v $_HOME_/"$system_to_build_for"/artefacts:/artefacts \
      -v $_HOME_/"$system_to_build_for"/script:/script \
      -v $_HOME_/"$system_to_build_for"/workspace:/workspace \
      --net=host \
     "ffmpeg_av_jni_ready_ub22_win_001" \
     /bin/sh -c "apk add bash >/dev/null 2>/dev/null; /bin/bash /script/run.sh"
     if [ $? -ne 0 ]; then
        echo "** ERROR **:$system_to_build_for_orig"
        exit 1
     else
        echo "--SUCCESS--:$system_to_build_for_orig"
     fi

done


