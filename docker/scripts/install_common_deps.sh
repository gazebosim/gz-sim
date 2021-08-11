#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get update

sudo apt-get install --no-install-recommends -y \
  gnupg \
  lsb-release \
  software-properties-common \
  wget

sudo apt-get install --no-install-recommends -y \
  build-essential \
  cmake \
  cppcheck \
  curl \
  git \
  g++-8 \
  pkg-config \

sudo apt-get install --no-install-recommends -y \
  clang-tidy-6.0 \
  python-yaml \
  libclang-6.0-dev

sudo apt-get install --no-install-recommends -y \
  libbenchmark-dev \
  libbenchmark1

sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
