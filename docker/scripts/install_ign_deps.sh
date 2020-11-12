#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get update

# Things that are used all over the ign stack
sudo apt-get install --no-install-recommends -y \
  doxygen \
  libbullet-dev \
  libtinyxml2-dev \
  libprotoc-dev libprotobuf-dev \
  protobuf-compiler \
  ruby-ronn \
  ruby-dev \
  swig \
  uuid-dev

# ign-common dependencies
sudo apt-get install --no-install-recommends -y \
  libavcodec-dev \
  libavdevice-dev \
  libavformat-dev \
  libavutil-dev \
  libfreeimage-dev \
  libgts-dev \
  libswscale-dev

# ign-gui dependencies
sudo apt-get install --no-install-recommends -y \
  qtbase5-dev \
  qtdeclarative5-dev \
  qtquickcontrols2-5-dev \
  qml-module-qtquick2 \
  qml-module-qtquick-controls \
  qml-module-qtquick-controls2 \
  qml-module-qtquick-dialogs \
  qml-module-qtquick-layouts \
  qml-module-qt-labs-folderlistmodel \
  qml-module-qt-labs-settings \
  qml-module-qtgraphicaleffects

# ign-rendering dependencies
sudo apt-get install --no-install-recommends -y \
  libogre-1.9-dev \
  libogre-2.1-dev \
  libglew-dev \
  libfreeimage-dev \
  freeglut3-dev \
  libxmu-dev \
  libxi-dev

# ign-transport dependencies
sudo apt-get install --no-install-recommends -y \
  libzmq3-dev \
  libsqlite3-dev

# SDFormat dependencies
sudo apt-get install --no-install-recommends -y \
  libtinyxml-dev libxml2-dev

# ign-fuel_tools dependencies
sudo apt-get install --no-install-recommends -y \
  libcurl4-openssl-dev libjsoncpp-dev libzip-dev curl libyaml-dev

# ign-physics dependencies
sudo apt-get install --no-install-recommends -y \
  libeigen3-dev \
  dart6-data \
  libdart6-collision-ode-dev \
  libdart6-dev \
  libdart6-utils-urdf-dev \
  libbenchmark-dev

# ign-gazebo dependencies
sudo apt-get install --no-install-recommends -y \
  qml-module-qtqml-models2

sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

