#!/bin/sh -l

set -x
set -e

# Install (needed for some tests)
make install
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
