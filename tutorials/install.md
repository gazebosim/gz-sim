\page install Installation

These instructions are for installing only Gazebo Sim. If you're interested
in using all the Gazebo libraries including gz-launch, or to install all
libraries from source, check out
[Gazebo installation](https://gazebosim.org/docs/latest/install).

We recommend following the binary install instructions to get up and running as
quickly and painlessly as possible.

The source install instructions should be used if you need the very latest
software improvements, if you need to modify the code, or if you plan to make a
contribution.

Throughout this tutorial, be sure to replace `<#>` with a number value, such as
5 or 6, depending on which version you need.

## Binary Install

### Ubuntu 20.04 or above

The binary install method will use pre-built packages which are typically
available through a package management utility such as [Apt](https://wiki.debian.org/Apt).
This approach eliminates the need to download and compile source code, and dependencies
are handled for you. The downside of a binary install is that you won't be able to modify
the code. See [Source Install](#source-install) for information on
installing Gazebo from source.

1. Configure package repositories.
  ```
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update
  ```

2. Install Gazebo
  ```
  sudo apt-get install libgz-sim<#>-dev
  ```

### macOS

1. On macOS, add OSRF packages:
  ```
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  brew tap osrf/simulation
  ```

2. Install Gazebo:
  ```
  brew install gz-sim<#>
  ```

### Windows

1. First, follow the steps 1-4 of [Gazebo source install](https://gazebosim.org/docs/ionic/install_windows_src/#install-dependencies)
  tutorial to install Conda, Visual Studio and set up a Conda environment (we'll call it `gz-ws`).

  Do not forget to activate the Conda env (if not already activated)
  inside the Visual Studio Command Prompt:

  ```
  conda activate gz-ws
  ```

2. Now install Gazebo.

  ```
  conda install gz-sim<#> --channel conda-forge
  ```

  Be sure to replace `<#>` with a number value, such as 8 or 9, depending on
  which version you need.

## Source Install

Install from source if you're interested in changing the source code or need a
feature which hasn't been released yet.

### Ubuntu 20.04 or above

1. Install tools
  ```
  sudo apt install -y build-essential cmake git gnupg lsb-release wget
  ```

2. Enable the Gazebo software repositories:
  ```
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update
  ```

3. Clone repository
  ```
  git clone https://github.com/gazebosim/gz-sim -b gz-sim<#>
  ```

4. Install package dependencies (including other Gazebo libraries):
  ```
  sudo apt -y install \
    $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | tr '\n' ' '))
  ```

5. Configure and build.
  ```
  cd gz-sim
  mkdir build
  cd build
  cmake ../
  make
  ```

### macOS

1. Clone the repository
  ```
  git clone https://github.com/gazebosim/gz-sim -b gz-sim<#>
  ```

2. Install dependencies
  ```
  brew install --only-dependencies gz-sim<#>
  ```

3. Configure and build
  ```
  cd gz-sim
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

1. First, follow the steps 1-4 of [Gazebo source install](https://gazebosim.org/docs/ionic/install_windows_src/#install-dependencies)
  tutorial to install Conda, Visual Studio and set up a Conda environment (we'll call it `gz-ws`).

  Do not forget to activate the Conda env (if not already activated)
  inside the Visual Studio Command Prompt:

  ```
  conda activate gz-ws
  ```

2. Install Gazebo dependencies:

  You can view available versions and their dependencies:
  ```
  conda search libgz-sim* --channel conda-forge --info
  ```

  Install the dependencies, replacing `<#>` with the required versions:
  ```
  conda install gz-cmake<#> gz-common<#> gz-fuel-tools<#> gz-gui<#> gz-math<#> gz-msgs<#> gz-physics<#> gz-plugin<#> gz-rendering<#> gz-sensors<#> gz-tools<#> gz-transport<#> gz-utils<#> sdformat<#> --channel conda-forge
  ```

3. Navigate to where you would like to build the library, and clone the repository.
  ```
  # Optionally, append `-b gz-sim#` (replace # with a number) to check out a specific version
  git clone https://github.com/gazebosim/gz-sim.git
  ```

4. Configure and build
  ```
  cd gz-sim
  mkdir build
  cd build
  cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
  cmake --build . --config Release
  ```

5. Optionally, install
  ```
  cmake --install . --config Release
  ```

## Documentation

API documentation and tutorials can be accessed at [https://gazebosim.org/libs/gazebo](https://gazebosim.org/libs/gazebo)

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need [Doxygen](http://www.doxygen.org/). On Ubuntu Doxygen can be installed using
  ```
  # Linux
  sudo apt-get install doxygen
  # Windows
  conda install doxygen --channel conda-forge
  ```

2. Clone the repository
  ```
  git clone https://github.com/gazebosim/gz-sim
  ```

3. Configure and build the documentation.
  ```
  cd gz-sim
  mkdir build
  cd build
  cmake ../
  cmake --build . --target doc
  ```

4. View the documentation by running the following command from the `build` directory.
  ```
  firefox doxygen/html/index.html
  ```

## Testing

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
