#!/bin/sh -l

set -x

# Install (needed for some tests)
make install

# For ign-tools
export IGN_CONFIG_PATH=/usr/local/share/ignition
