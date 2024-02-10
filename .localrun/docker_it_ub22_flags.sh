#! /bin/bash

_HOME2_=$(dirname $0)
export _HOME2_
_HOME_=$(cd $_HOME2_;pwd)
export _HOME_

echo $_HOME_
cd $_HOME_

if [ "$1""x" == "buildx" ]; then
    docker build -f Dockerfile_ub22 -t ffmpeg_av_jni_ready_ub22_001 .
    exit 0
fi

build_for='
ubuntu:22.04
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
ls -al "$_INST2_"/bin/

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



## -------------------------------
## -------------------------------
## -------------------------------
## -------------------------------
## -------------------------------
## -------------------------------

add_config_flag() { CONFIG_FLAGS+=("$@"); }
add_c_flag() { C_FLAGS="$C_FLAGS $@"; }
add_cxx_flag() { CXX_FLAGS="$CXX_FLAGS $@"; }
add_ld_flag() { LD_FLAGS="$LD_FLAGS $@"; }
add_flag() {
  add_c_flag "$@"
  add_cxx_flag "$@"
}

CONFIG_FLAGS=()
C_FLAGS=""
CXX_FLAGS=""
LD_FLAGS=""

unset CFLAGS
unset CXXFLAGS
unset CPPFLAGS
unset LDFLAGS

# Optimisation flags.
add_flag -O3 -march=native

# Warn on non-ISO C.
add_c_flag -pedantic
add_c_flag -std=gnu99
add_cxx_flag -std=c++11

add_flag -g3
add_flag -ftrapv

## ...............................

# Add all warning flags we can.
add_flag -Wall
add_flag -Wextra

# Some additional warning flags not enabled by any of the above.
add_flag -Wbool-compare
add_flag -Wcast-align
add_flag -Wcast-qual
add_flag -Wchar-subscripts
add_flag -Wdouble-promotion
add_flag -Wduplicated-cond
add_flag -Wempty-body
add_flag -Wenum-compare
add_flag -Wfloat-equal
add_flag -Wformat=2
add_flag -Wframe-address
add_flag -Wframe-larger-than=9000
add_flag -Wignored-attributes
add_flag -Wignored-qualifiers
add_flag -Winit-self
add_flag -Winline
add_flag -Wlarger-than=530000
add_flag -Wmaybe-uninitialized
add_flag -Wmemset-transposed-args
add_flag -Wmisleading-indentation
add_flag -Wmissing-declarations
add_flag -Wnonnull
add_flag -Wnull-dereference
add_flag -Wodr
add_flag -Wredundant-decls
add_flag -Wreturn-type
add_flag -Wshadow
add_flag -Wsuggest-attribute=format
add_flag -Wundef
add_flag -Wunsafe-loop-optimizations
add_flag -Wunused-but-set-parameter
add_flag -Wunused-but-set-variable
add_flag -Wunused-label
add_flag -Wunused-local-typedefs
add_flag -Wunused-value


add_flag -Wno-missing-field-initializers
add_flag -Wno-sign-compare
add_flag -Wno-type-limits
add_flag -Wno-unused-parameter
add_flag -Wno-unused-function
add_flag -Wno-missing-braces
add_flag -Wno-nonnull-compare

## ...............................

add_flag -Werror
add_flag -fdiagnostics-color=always
add_flag -fno-omit-frame-pointer
add_flag -Wno-error=declaration-after-statement
#add_flag -Wno-error=documentation

add_flag -Wno-error=unused-but-set-variable
#add_flag -Wno-error=c11-extensions

#add_flag -Wno-error=missing-variable-declarations
add_flag -Wno-error=missing-prototypes
#add_flag -Wno-error=embedded-directive
add_flag -Wno-error=unused-macros

#add_flag -Wno-error=unsafe-buffer-usage
#add_flag -Wno-error=cast-function-type-strict

add_flag -Wno-error=missing-declarations
add_flag -Wno-error=unused-const-variable=
add_flag -Wno-error=deprecated-declarations
add_flag -Wno-error=discarded-qualifiers

## -------------------------------
## -------------------------------
## -------------------------------
## -------------------------------
## -------------------------------
## -------------------------------


export CFLAGS_ADD=" -fPIC -I$_INST2_/include/ -L$_INST2_/lib -fstack-protector-all -D_FORTIFY_SOURCE=2 "


faketime "2023-10-10 08:00:00" gcc $C_FLAGS $CFLAGS_ADD \
-DJAVA_LINUX \
-D_FILE_OFFSET_BITS=64 -D__USE_GNU=1 \
-I$JAVADIR1/ \
-I$JAVADIR2/ \
ffmpeg_av_jni.c \
$_INST2_/lib/libavcodec.a \
$_INST2_/lib/libavdevice.a \
$_INST2_/lib/libavformat.a \
$_INST2_/lib/libavutil.a \
$(pkg-config --libs --cflags libavformat) \
$(pkg-config --libs --cflags libavfilter) \
$(pkg-config --libs --cflags libavcodec) \
$(pkg-config --libs --cflags libv4l2) \
$(pkg-config --libs --cflags xcb) \
$(pkg-config --libs --cflags xcb-shm) \
$(pkg-config --libs --cflags libswscale) \
$(pkg-config --libs --cflags libpulse) \
$(pkg-config --libs --cflags alsa) \
$(pkg-config --libs --cflags libswresample) \
-lpthread \
-lm \
-shared \
-Wl,-soname,libffmpeg_av_jni.so -o libffmpeg_av_jni.so || exit 1


exit 1

echo "compiling test program ..."

gcc $CFLAGS \
-Wall \
-fsanitize=address -fno-omit-frame-pointer -fsanitize-recover=address -static-libasan \
-Wno-unused-function \
-Wno-discarded-qualifiers \
-Wno-unused-const-variable \
-Wno-deprecated-declarations \
-D_FILE_OFFSET_BITS=64 -D__USE_GNU=1 \
test.c \
$_INST2_/lib/libavcodec.a \
$_INST2_/lib/libavdevice.a \
$_INST2_/lib/libavformat.a \
$_INST2_/lib/libavutil.a \
$(pkg-config --libs --cflags libavformat) \
$(pkg-config --libs --cflags libavcodec) \
$(pkg-config --libs --cflags libv4l2) \
$(pkg-config --libs --cflags xcb) \
$(pkg-config --libs --cflags xcb-shm) \
$(pkg-config --libs --cflags libswscale) \
$(pkg-config --libs --cflags libpulse) \
$(pkg-config --libs --cflags alsa) \
$(pkg-config --libs --cflags libswresample) \
$(pkg-config --libs --cflags xcb-xfixes) \
$(pkg-config --libs --cflags x11-xcb) \
$(pkg-config --libs --cflags xcb-shape) \
-lpthread \
-lm \
-o test
ls -al test
cp -av test /artefacts/





cp -av "$_INST2_"/bin/* /artefacts/

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
     "ffmpeg_av_jni_ready_ub22_001" \
     /bin/sh -c "apk add bash >/dev/null 2>/dev/null; /bin/bash /script/run.sh"
     if [ $? -ne 0 ]; then
        echo "** ERROR **:$system_to_build_for_orig"
        exit 1
     else
        echo "--SUCCESS--:$system_to_build_for_orig"
     fi

done


