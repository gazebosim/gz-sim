#!/bin/bash

set -o errexit
set -o verbose

mkdir build

cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX=/usr \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=1

make -j6

sudo make install
