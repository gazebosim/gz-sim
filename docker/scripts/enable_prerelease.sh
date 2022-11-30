#!/bin/bash

set -o errexit
set -o verbose

sudo echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
