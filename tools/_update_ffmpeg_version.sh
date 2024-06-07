#! /bin/bash

ver="n7.0.1"

cd ../ci_scripts/
grep -ril 'FFMPEG_VERSION='|xargs -L1 sed -i -e 's#FFMPEG_VERSION="[^"]*"#FFMPEG_VERSION="'"$ver"'"#'
cd ../.localrun/
grep -ril 'FFMPEG_VERSION='|xargs -L1 sed -i -e 's#FFMPEG_VERSION="[^"]*"#FFMPEG_VERSION="'"$ver"'"#'

