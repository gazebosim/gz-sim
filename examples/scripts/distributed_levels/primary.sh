#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

ign-gazebo -v 4 --levels --network-role=primary --network-secondaries=2 -f $DIR/primary.sdf

# Ignition Gazebo 1.x.x and 2.x.x support using environment variables to
# configure distributed simulation. This capability is deprecated in
# version 2.x.x, and removed in verion 3.x.x of Inition Gazebo. Please use the
# --network-role and --network-secondaries command line options instead.

# export IGN_GAZEBO_NETWORK_ROLE="PRIMARY"
# export IGN_GAZEBO_NETWORK_SECONDARIES=3
# ign-gazebo -v 4 --distributed -f $DIR/primary.sdf


