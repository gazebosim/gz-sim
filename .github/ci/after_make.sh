#!/bin/sh -l

set -x
set -e

# Install (needed for some tests)
make install

# For ign-tools
export IGN_CONFIG_PATH=/usr/local/share/ignition

# For rendering / window tests
Xvfb :1 -screen 0 1280x1024x24 &
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3
