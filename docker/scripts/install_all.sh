#!/bin/bash

set -o errexit
set -o verbose


# Install base dependencies
./docker/scripts/install_common_deps.sh
./docker/scripts/install_ign_deps.sh
./docker/scripts/enable_gcc8.sh
# Enable relevant package repositories
./docker/scripts/enable_ign_stable.sh
./docker/scripts/enable_ign_prerelease.sh

# Install Ignition libraries from Debian.
./docker/scripts/install_ign.sh
