#!/bin/bash

set -o errexit
set -o verbose

# Install base dependencies
./docker/scripts/install_common_deps.sh
./docker/scripts/enable_gcc8.sh

./docker/scripts/install_ign_deps.sh

# Install Ignition libraries from Debian.
./docker/scripts/install_ign.sh
