#! /bin/bash

_HOME_="$(pwd)"
export _HOME_

cd "$_HOME_"

export _SRC_=$_HOME_/src/
export _INST_=$_HOME_/inst/

mkdir -p $_SRC_
mkdir -p $_INST_

export LD_LIBRARY_PATH=$_INST_/lib/
export PKG_CONFIG_PATH=$_INST_/lib/pkgconfig


# -- more Deterministic builds --
date
export TZ=UTC
date
locale
export LC_ALL=C
export LANG=C
export LANGUAGE=C
locale
export ZERO_AR_DATE=1
export SOURCE_DATE_EPOCH=1696924800
# -- more Deterministic builds --


# ----------- config ------------
ORIGPATH=$PATH
export ORIGPATH
NEWPATH=$PATH # /usr/x86_64-w64-mingw32/bin:$PATH
export NEWPATH
export PATH=$NEWPATH

MAKEFLAGS=j$(nproc)
export MAKEFLAGS

WGET_OPTIONS="--timeout=10"
export WGET_OPTIONS

CXXFLAGS_ADDON='-O2 -g -fPIC --param=ssp-buffer-size=1 -fstack-protector-all -D_FORTIFY_SOURCE=2'
CFLAGS_ADDON='-O2 -g -fPIC --param=ssp-buffer-size=1 -fstack-protector-all -D_FORTIFY_SOURCE=2'
# ----------- config ------------


# ------- deps verisions ---------
FFMPEG_VERSION="n7.1"
# ------- deps verisions ---------



# ---------- ffmpeg ---------
if [ 1 == 1 ]; then

cd "$_SRC_"

FFMPEG_FILENAME="$FFMPEG_VERSION.tar.gz"
rm -f "ffmpeg"*.tar.*
wget $WGET_OPTIONS "https://github.com/FFmpeg/FFmpeg/archive/refs/tags/$FFMPEG_FILENAME" -O "ffmpeg_""$FFMPEG_FILENAME"
tar -xf "ffmpeg_""$FFMPEG_FILENAME"
rm -f "ffmpeg"*.tar.*
cd *mpeg*/

export LDFLAGS=" "

#
# sadly need "--disable-asm" or you will get:
#
# libavcodec.a(h264_qpel_10bit.o): relocation R_X86_64_PC32 against symbol `ff_pw_1023' can not be used when making a shared object; recompile with -fPIC
#

  export CXXFLAGS=${CXXFLAGS_ADDON}
  export CFLAGS=${CFLAGS_ADDON}

  if [ "$1""x" == "raspix" ]; then
    echo "*** RASPI ***"
  ./configure --arch="aarch64" \
              --enable-gpl \
              --prefix="$_INST_" \
              --target-os="linux" \
              --cross-prefix="$CROSS_COMPILE" \
              --disable-asm \
              --enable-pic \
              --disable-network \
              --disable-everything \
              --disable-debug \
              --disable-shared \
              --disable-protocols \
              --disable-doc \
              --disable-sdl2 \
              --disable-filters \
              --disable-iconv \
              --disable-network \
              --disable-postproc \
              --disable-swscale-alpha \
              --disable-dwt \
              --disable-lsp \
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
              --enable-pic \
              --enable-swscale \
              --enable-swresample \
              --enable-avcodec \
              --enable-decoder=mjpeg \
              --enable-demuxer=rawvideo \
              --enable-decoder=rawvideo \
              --enable-demuxer=pcm_s16le \
              --enable-decoder=pcm_s16le \
              --enable-filter=volume \
              --enable-filter=speechnorm \
              --enable-filter=loudnorm \
              --enable-filter=arnndn \
              --enable-filter=afftdn \
              --enable-filter=aresample || exit 1

  else

  ./configure \
              --enable-gpl \
              --prefix="$_INST_" \
              --enable-asm \
              --extra-cflags="-fdebug-prefix-map=/home/runner/work/ffmpeg_av_jni/ffmpeg_av_jni=/ -static -O2 -g0 -D_FORTIFY_SOURCE=2 -fstack-protector-strong" \
              --extra-ldflags="-fdebug-prefix-map=/home/runner/work/ffmpeg_av_jni/ffmpeg_av_jni=/ -lm -fstack-protector-strong" \
              --disable-network \
              --disable-everything \
              --disable-debug \
              --disable-shared \
              --disable-protocols \
              --disable-doc \
              --disable-sdl2 \
              --disable-filters \
              --disable-iconv \
              --disable-network \
              --disable-postproc \
              --disable-swscale-alpha \
              --disable-dwt \
              --disable-lsp \
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
              --enable-pic \
              --enable-swscale \
              --enable-swresample \
              --enable-avcodec \
              --enable-outdev=alsa \
              --enable-outdev=pulse \
              --enable-indev=alsa \
              --enable-indev=xcbgrab \
              --enable-decoder=mjpeg \
              --enable-demuxer=rawvideo \
              --enable-decoder=rawvideo \
              --enable-demuxer=pcm_s16le \
              --enable-decoder=pcm_s16le \
              --enable-libxcb --enable-libxcb-shm --enable-libxcb-xfixes --enable-libxcb-shape \
              --enable-indev=pulse \
              --enable-indev=v4l2 \
              --enable-libpulse \
              --enable-filter=volume \
              --enable-filter=speechnorm \
              --enable-filter=loudnorm \
              --enable-filter=arnndn \
              --enable-filter=afftdn \
              --enable-filter=aresample \
              --enable-runtime-cpudetect \
              --enable-libv4l2 || exit 1

  fi


#              --disable-lzo \
#              --disable-avresample \


  make -j || exit 1
  make install

  unset CXXFLAGS
  unset CFLAGS

cd "$_HOME_"

ls -al "$_INST_"/lib/

fi
# ---------- ffmpeg ---------

