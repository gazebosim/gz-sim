#!/bin/sh -l

set -x

# For rendering / window tests
Xvfb :1 -screen 0 1280x1024x24 &
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3

# Fail early if Zenoh is requested but gz-transport was built without it.
# Otherwise, leave a marker in the build folder so the workflow can check
# that the variable reached this container.
if [ "$GZ_TRANSPORT_IMPLEMENTATION" = "zenoh" ]; then
  if ! grep -qs '#define HAVE_ZENOH 1' /usr/include/gz/transport*/gz/transport/config.hh; then
    echo "GZ_TRANSPORT_IMPLEMENTATION=zenoh but gz-transport has no Zenoh support"
    exit 1
  fi
  echo zenoh > gz_transport_implementation.txt
fi
