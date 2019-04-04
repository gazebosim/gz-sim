#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get update

# Things that are used all over the ign stack
sudo apt-get install -y \
  libignition-cmake2-dev \
  libignition-common3-dev \
  libignition-gui-dev \
  libignition-math6-eigen3-dev \
  libignition-msgs3-dev \
  libignition-plugin-dev \
  libignition-physics-dev \
  libignition-tools-dev \
  libignition-transport6-dev \
  libsdformat8-dev \
  libignition-fuel-tools3-dev
  # libignition-rendering-dev
  # libignition-sensors-dev
