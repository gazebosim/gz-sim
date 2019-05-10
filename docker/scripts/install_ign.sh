#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get update

# Things that are used all over the ign stack
sudo apt-get install -y \
  libignition-cmake2-dev \
  libignition-common3-dev \
  libignition-fuel-tools3-dev \
  libignition-gui2-dev \
  libignition-math6-eigen3-dev \
  libignition-msgs4-dev \
  libignition-plugin-dev \
  libignition-physics-dev \
  libignition-rendering2-dev \
  libignition-sensors2-dev \
  libignition-tools-dev \
  libignition-transport7-dev \
  libsdformat8-dev
