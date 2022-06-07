#!/bin/bash
# Command line parameters:
# 1 - github organization name. For example gazebosim or osrf.
# 2 - the name of the Gazebo repository. For example gz-math.
# 3 - the name of the branch. For example gz-math7

set -o errexit
set -o verbose

git clone https://github.com/$1/$2 -b $3
cd $2
mkdir build
cd build
cmake ../ -DBUILD_TESTING=false
sudo make -j4 install
