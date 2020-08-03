#!/bin/sh -l

set -x

BUILD_DIR=`pwd`

# Install
make install

# Compile examples
cd ../examples
mkdir build
cd build
cmake ..
make
./graph_example

cd $BUILD_DIR
