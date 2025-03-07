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
NEWPATH=/usr/x86_64-w64-mingw32/bin:$PATH
export NEWPATH
export PATH=$NEWPATH

MAKEFLAGS=j$(nproc)
export MAKEFLAGS

WGET_OPTIONS="--timeout=10"
export WGET_OPTIONS

ARCH="x86_64"
export ARCH

export CXXFLAGS_ADDON="-O2 -g -fPIC"
export CFLAGS_ADDON="-O2 -g -fPIC"
export CFLAGS_ADDON_MORE="--param=ssp-buffer-size=1 -fstack-protector-all"
# ----------- config ------------


# ------- deps verisions ---------
FFMPEG_VERSION="n7.1.1"
NASM_VERSION="nasm-2.16.01"
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

sed -i -e 's#die "Host compiler lacks C11 support"#echo "Host compiler lacks C11 support"#' configure

  ./configure --arch="$ARCH" \
              --enable-gpl \
              --prefix="$_INST_" \
              --enable-asm \
              --target-os="mingw32" \
              --cross-prefix="$ARCH-w64-mingw32-" \
              --pkg-config="pkg-config" \
              --extra-cflags="-static -O2 -g0 -D_FORTIFY_SOURCE=2 -fstack-protector-strong" \
              --extra-ldflags="-lm -static -fstack-protector-strong" \
              --pkg-config-flags="--static" \
              --disable-network \
              --disable-debug \
              --disable-shared \
              --disable-protocols \
              --disable-doc \
              --disable-sdl2 \
              --disable-filters \
              --disable-iconv \
              --disable-network \
              --disable-muxers \
              --disable-postproc \
              --disable-swscale-alpha \
              --disable-dwt \
              --disable-lsp \
              --disable-faan \
              --disable-vaapi \
              --disable-vdpau \
              --disable-zlib \
              --disable-xlib \
              --disable-bzlib \
              --disable-lzma \
              --disable-encoders \
              --disable-decoders \
              --disable-demuxers \
              --disable-parsers \
              --disable-bsfs \
              --enable-swscale \
              --enable-swresample \
              --enable-indev=openal \
              --enable-indev=dshow \
              --enable-indev=gdigrab \
              --enable-indev=vfwcap \
              --enable-decoder=mjpeg \
              --enable-decoder=bmp \
              --enable-demuxer=rawvideo \
              --enable-decoder=rawvideo \
              --enable-demuxer=pcm_s16le \
              --enable-decoder=pcm_s16le \
              --enable-filter=volume \
              --enable-filter=speechnorm \
              --enable-filter=loudnorm \
              --enable-filter=arnndn \
              --enable-filter=afftdn \
              --enable-filter=aresample \
              --enable-runtime-cpudetect || exit 1

#              --disable-lzo \
#              --disable-avresample \

  make -j || exit 1
  make install

cd "$_HOME_"

ls -al "$_INST_"/lib/

fi
# ---------- ffmpeg ---------

