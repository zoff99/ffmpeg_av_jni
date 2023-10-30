#! /bin/bash

_HOME2_=$(dirname $0)
export _HOME2_
_HOME_=$(cd $_HOME2_;pwd)
export _HOME_

echo $_HOME_
cd $_HOME_

_HOME_="$(pwd)"
export _HOME_

cd "$_HOME_"

cd ../
./ci_scripts/deps_macos.sh
./ci_scripts/java_jni_lib_macos.sh
