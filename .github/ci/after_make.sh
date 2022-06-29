#!/bin/sh -l

set -x
set -e

# Install (needed for some tests)
make install
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

# For gz-tools
export GZ_CONFIG_PATH=/usr/local/share/ignition

# For rendering / window tests
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3
