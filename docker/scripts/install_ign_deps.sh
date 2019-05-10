#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get update

# Things that are used all over the ign stack
sudo apt-get install -y \
  doxygen \
  libbullet-dev \
  libeigen3-dev \
  libtinyxml2-dev \
  libprotoc-dev libprotobuf-dev \
  protobuf-compiler \
  ruby-ronn \
  ruby-dev \
  swig \
  uuid-dev

# ign-common dependencies
sudo apt-get install -y \
  libavcodec-dev \
  libavdevice-dev \
  libavformat-dev \
  libavutil-dev \
  libfreeimage-dev \
  libgts-dev \
  libswscale-dev

# ign-gui dependencies
sudo apt-get install -y \
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
sudo apt-get install -y \
  libogre-1.9-dev \
  libogre-2.1-dev \
  libglew-dev \
  libfreeimage-dev \
  freeglut3-dev \
  libxmu-dev \
  libxi-dev

# ign-transport dependencies
sudo apt-get install -y \
  libzmq3-dev \
  libsqlite3-dev

# SDFormat dependencies
sudo apt-get install -y \
  libxml2-dev

# ign-gazebo dependencies
sudo apt-get install -y \
  libgflags-dev \
  qml-module-qtqml-models2

sudo apt-get clean

