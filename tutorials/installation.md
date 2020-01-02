\page install Installation

Next Tutorial: \ref cppgetstarted

## Overview

This tutorial describes how to install Ignition Math on Linux, Mac OS X and
Windows via either a binary distribution or from source.

## Ubuntu Linux

Setup your computer to accept software from
*packages.osrfoundation.org*:

```{.sh}
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:

```{.sh}
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install Ignition Math:

```
sudo apt-get update
sudo apt-get install libignition-math6-dev
```
