#!/usr/bin/env bash

set -xe

mkdir -p build_tidy
cd build_tidy
cmake .. \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-8 \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-8 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=1

# Build enough to get generated msg headers
make gz-sim_private_msgs

cd ..

run-clang-tidy-6.0.py \
  -p=`pwd`/build_tidy \
  -header-filter="`pwd`/(include|src)/*" \
  -j 6 \
  -quiet \
  "`pwd`/src/*" \
  "`pwd`/test/(benchmark|integration|performance|plugins|regression)/*" \
  "`pwd`/examples/*"
