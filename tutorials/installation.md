\page install Installation

Next Tutorial: \ref cppgetstarted

These instructions are for installing only Ignition Math.
If you're interested in using all the Ignition libraries, check out this [Ignition installation](https://ignitionrobotics.org/docs/latest/install).

We recommend following the Binary Installation instructions to get up and running as quickly and painlessly as possible.

The Source Installation instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

# Binary Installation

## Ubuntu Linux

Setup your computer to accept software from
*packages.osrfoundation.org*:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install Ignition Math:
```
sudo apt install libignition-math<#>-dev
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

### macOS

On macOS, add OSRF packages:
  ```
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  brew tap osrf/simulation
  ```

Install Ignition Math:
  ```
  brew install ignition-math<#>
  ```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

## Windows

Install [Conda package management system](https://docs.conda.io/projects/conda/en/latest/user-guide/install/download.html).
Miniconda suffices.

Create if necessary, and activate a Conda environment:
```
conda create -n ign-ws
conda activate ign-ws
```

Install:
```
conda install libignition-math<#> --channel conda-forge
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

# Source Installation

Source installation can be performed by first installing the necessary
prerequisites followed by building from source.

## Prerequisites

Ignition Math requires:

* [Ignition CMake](https://ignitionrobotics.org/libs/cmake)

### Ubuntu Linux

The optional Eigen component of Ignition Math requires:

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). Refer to the [Eigen Documentation](http://eigen.tuxfamily.org/index.php?title=Main_Page#Documentation) for installation instructions. On Ubuntu systems, `apt-get` can be used to install Eigen:
  ```
  sudo apt-get install libeigen3-dev
  ```

The optional Ruby tests of Ignition Math require:

* [Ruby](https://www.ruby-lang.org/). Refer to the [Ruby Documentation](https://www.ruby-lang.org/downloads/) for installation instructions. On Ubuntu systems `apt-get` can be used to install Ubuntu Package `ruby-dev`:
  ```
  sudo apt-get install ruby-dev
  ```

* [Swig](http://www.swig.org/). Refer to the [Swig Documentation](http://www.swig.org/download.html) for installation instructions. On Ubuntu systems `apt-get` can be used to install Swig:
  ```
  sudo apt-get install swig
  ```

### Windows 10

First, follow the [ign-cmake](https://github.com/ignitionrobotics/ign-cmake) tutorial for installing Conda, Visual Studio, CMake, and other prerequisites, and also for creating a Conda environment.

The optional Eigen component of Ignition Math requires:

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). Refer to the [Eigen Documentation](http://eigen.tuxfamily.org/index.php?title=Main_Page#Documentation) for installation instructions. On Windows, we will use `conda` to install Eigen:
  ```
  conda install eigen --channel conda-forge
  ```

## Building from Source

### Ubuntu

1. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-math -b ign-math<#>
  ```
  Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
  which version you need.

2. Install dependencies
  ```
  export SYSTEM_VERSION=bionic
  sudo apt -y install \
    $(sort -u $(find . -iname 'packages-'$SYSTEM_VERSION'.apt' -o -iname 'packages.apt') | tr '\n' ' ')
  ```

3. Configure and build
  ```
  cd ign-math
  mkdir build
  cd build
  cmake ..
  make
  ```

4. Optionally, install
  ```
  sudo make install
  ```

### macOS

1. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-math -b ign-math<#>
  ```
  Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
  which version you need.

2. Install dependencies
  ```
  brew install --only-dependencies ignition-math<#>
  ```
  Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
  which version you need.

3. Configure and build
  ```
  cd ign-math
  mkdir build
  cd build
  cmake ..
  make
  ```

4. Optionally, install
  ```
  sudo make install
  ```

### Windows

1. Navigate to `condabin` if necessary to use the `conda` command (i.e., if Conda is not in your `PATH` environment variable. You can find the location of `condabin` in Anaconda Prompt, `where conda`).
  Activate the Conda environment created in the prerequisites:
  ```
  conda activate ign-ws
  ```

2. Install dependencies

  You can view available versions and their dependencies:
  ```
  conda search libignition-math* --channel conda-forge --info
  ```
  See the [Conda release repository](https://github.com/conda-forge/libignition-math4-feedstock) for more information.

  Install dependencies, replacing `<#>` with the desired version:
  ```
  conda install libignition-cmake<#> --channel conda-forge
  ```

3. Navigate to where you would like to build the library, and clone the repository.
  ```
  # Optionally, append `-b ign-math#` (replace # with a number) to check out a specific version
  git clone https://github.com/ignitionrobotics/ign-math.git
  ```

4. Configure and build
  ```
  cd ign-math
  mkdir build
  cd build
  cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
  cmake --build . --config Release
  ```

5. Optionally, install
  ```
  cmake --install . --config Release
  ```

# Documentation

API and tutorials can be found at [https://ignitionrobotics.org/libs/math](https://ignitionrobotics.org/libs/math).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using
 ```
 sudo apt-get install doxygen
 ```

2. Clone the repository
 ```
 git clone https://github.com/ignitionrobotics/ign-math
 ```

3. Configure and build the documentation.
 ```
 cd ign-math; mkdir build; cd build; cmake ../; make doc
 ```

4. View the documentation by running the following command from the build directory.
 ```
 firefox doxygen/html/index.html
 ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Follow the [source install instruction](https://ignitionrobotics.org/libs/math#source-install).

2. Run tests.
 ```
 make test
 ```

3. Static code checker.
 ```
 make codecheck
 ```

## Ruby Tests

### Usage

The C++ classes are available in Ruby code by interface files (`.i`) used by swig to build a C++ extension module.

The interfaces and Ruby test codes are in the `src` folder. To use a C++ class in Ruby you need to:

1. Create an interface file describing the class as in Swig and Ruby reference at [The Ruby-to-C/C++ Mapping](http://www.swig.org/Doc1.3/Ruby.html#Ruby_nn11)

2. Include the interface file in `/src/ing_math.i`

3. Create the Ruby file and import the class as in Swig and Ruby reference at [C++ Classes](http://www.swig.org/Doc1.3/Ruby.html#Ruby_nn18)

### Tests

`make test` already runs all tests, including the ones made in Ruby, but you can run a Ruby test individually using
  ```
  ctest -R Ruby_TEST.rb
  ```
