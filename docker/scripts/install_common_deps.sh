#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get update

sudo apt-get install -y \
  gnupg \
  lsb-release \
  software-properties-common \
  wget

sudo apt-get install -y \
  build-essential \
  cmake \
  curl \
  git \
  g++-8 \
  mercurial \
  pkg-config

sudo apt-get install -y \
  cppcheck \
  valgrind

sudo apt-get install -y \
  python3 \
  python3-dbg \
  python3-pip \
  python3-venv
