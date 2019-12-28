#!/usr/bin/env bash

set -xe
shopt -s globstar

mkdir -p build_tidy
cd build_tidy
cmake .. \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-8 \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-8 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=1

# Build enough to get generated msg headers
make ignition-gazebo_private_msgs

cd ..

clang-tidy-6.0 \
  -p=`pwd`/build_tidy \
  -header-filter="`pwd`/(include|src)/*" \
  -quiet \
  -warnings-as-errors=* \
  `pwd`/src/*.cc \
  `pwd`/src/**/*.cc \
  `pwd`/test/{benchmark,integration,performance,plugins}/*.cc \
  `pwd`/examples/**/*.cc
