\page install Installation

# Install

These instructions are for installing only Ignition Gazebo. If you're interested
in using all the Ignition libraries, not only Igniton Gazebo, check out this
[Ignition installation](https://ignitionrobotics.org/docs/latest/install).

We recommend following the binary install instructions to get up and running as
quickly and painlessly as possible.

The source install instructions should be used if you need the very latest
software improvements, if you need to modify the code, or if you plan to make a
contribution.

Throughout this tutorial, be sure to replace `<#>` with a number value, such as
5 or 6, depending on which version you need.

## Binary Install

### Ubuntu 18.04 or above

The binary install method will use pre-built packages which are typically
available through a package management utility such as [Apt](https://wiki.debian.org/Apt).
This approach eliminates the need to download and compile source code, and dependencies
are handled for you. The downside of a binary install is that you won't be able to modify
the code. See [Source Install](#source-install) for information on
installing Ignition Gazebo from source.

1. Configure package repositories.
  ```
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update
  ```

2. Install Ignition Gazebo
  ```
  sudo apt-get install libignition-gazebo<#>-dev
  ```

### macOS

1. On macOS, add OSRF packages:
  ```
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  brew tap osrf/simulation
  ```

2. Install Ignition Gazebo:
  ```
  brew install ignition-gazebo<#>
  ```

## Source Install

Install from source if you're interested in changing the source code or need a
feature which hasn't been released yet.

### Ubuntu 18.04 or above

1. Install tools
  ```
  sudo apt install -y build-essential cmake git gnupg lsb-release wget
  ```

2. Enable the Ignition software repositories:
  ```
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update
  ```

3. Clone repository
  ```
  git clone https://github.com/ignitionrobotics/ign-gazebo -b ign-gazebo<#>
  ```

4. Install package dependencies (including other Ignition libraries):
  ```
  sudo apt -y install \
    $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | tr '\n' ' '))
  ```

5. (Ubuntu 18.04 only) Configure gcc8
  ```
  sudo apt-get install g++-8
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
  ```

6. Configure and build.
  ```
  cd ign-gazebo
  mkdir build
  cd build
  cmake ../
  make
  ```

### macOS

1. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-gazebo -b ign-gazebo<#>
  ```

2. Install dependencies
  ```
  brew install --only-dependencies ignition-gazebo<#>
  ```

3. Configure and build
  ```
  cd ign-gazebo
  mkdir build
  cd build
  cmake ..
  make
  ```

4. Optionally, install
  ```
  sudo make install
  ```

# Documentation

API documentation and tutorials can be accessed at [https://ignitionrobotics.org/libs/gazebo](https://ignitionrobotics.org/libs/gazebo)

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need [Doxygen](http://www.doxygen.org/). On Ubuntu Doxygen can be installed using
  ```
  sudo apt-get install doxygen
  ```

2. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-gazebo
  ```

3. Configure and build the documentation.
  ```
  cd ign-gazebo
  mkdir build
  cd build
  cmake ../
  make doc
  ```

4. View the documentation by running the following command from the `build` directory.
  ```
  firefox doxygen/html/index.html
  ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Follow the [source install instructions](#source-install).

2. Run tests.
  ```
  make test
  ```

3. Static code checker.
  ```
  sudo apt-get update && sudo apt-get -y install cppcheck
  make codecheck
  ```
