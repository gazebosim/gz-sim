#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get install g++-8

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8

gcc -v
g++ -v
gcov -v

# lcov
git clone https://github.com/linux-test-project/lcov.git -b v1.14
cd lcov
sudo make install
