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

CXXFLAGS_ADDON='-O2 -g -fPIC --param=ssp-buffer-size=1 -fstack-protector-all'
CFLAGS_ADDON='-O2 -g -fPIC --param=ssp-buffer-size=1 -fstack-protector-all'
# ----------- config ------------


# ------- deps verisions ---------
FFMPEG_VERSION="n6.0"
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
  ./configure \
              --enable-gpl \
              --prefix="$_INST_" \
              --disable-asm \
              --enable-pic \
              --enable-swscale \
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
              --enable-swresample \
              --enable-avcodec \
              --enable-outdev=alsa \
              --enable-outdev=pulse \
              --enable-indev=alsa \
              --enable-indev=xcbgrab \
              --enable-demuxer=rawvideo \
              --enable-decoder=rawvideo \
              --enable-demuxer=pcm_s16le \
              --enable-decoder=pcm_s16le \
              --enable-libxcb --enable-libxcb-shm --enable-libxcb-xfixes --enable-libxcb-shape \
              --enable-indev=pulse \
              --enable-indev=v4l2 \
              --enable-libpulse \
              --enable-libv4l2 || exit 1


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

