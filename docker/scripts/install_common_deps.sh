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
  g++ \
  lcov \
  pkg-config

sudo apt-get install --no-install-recommends -y \
  clang-tidy \
  python3-yaml

sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*
