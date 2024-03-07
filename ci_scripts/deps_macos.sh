#! /bin/bash

_SCRIPTDIR_=$(dirname $0)
export _SCRIPTDIR_
_SCRIPTDIR2_=$(cd $_SCRIPTDIR_;pwd)
export _SCRIPTDIR2_

_HOME_="$(pwd)"
export _HOME_

cd "$_HOME_"

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

export CXXFLAGS="$CXXFLAGS -fPIC"
export CFLAGS="$CFLAGS -fPIC"
# ----------- config ------------

type sudo

# ------- deps verisions ---------
FFMPEG_VERSION="n6.1.1"
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

  ./configure  \
              --enable-gpl \
              --prefix="$_INST_" \
              --disable-asm \
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
              --enable-indev=avfoundation \
              --enable-demuxer=rawvideo \
              --enable-decoder=rawvideo \
              --enable-demuxer=pcm_s16le \
              --enable-decoder=pcm_s16le \
              --enable-filter=volume \
              --enable-filter=arnndn \
              --enable-filter=afftdn \
              --enable-filter=aresample || exit 1

#              --enable-runtime-cpudetect || exit 1

  make -j || exit 1
  make install

cd "$_HOME_"

ls -al "$_INST_"/lib/

fi
# ---------- ffmpeg ---------




