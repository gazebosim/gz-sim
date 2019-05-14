#!/bin/bash
# Command line parameters:
# 1 - bitbucket organization name. For example ignitionrobotics or osrf.
# 2 - the name of the ignition repository. For example ign-math.
# 3 - the name of the branch. For example ign-math6

set -o errexit
set -o verbose

hg clone https://bitbucket.org/$1/$2 -b $3
cd $2 
mkdir build
cd build
cmake ../ -DBUILD_TESTING=false
sudo make -j4 install
