#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export IGN_GAZEBO_NETWORK_ROLE="PRIMARY"
export IGN_GAZEBO_NETWORK_SECONDARIES=2

# --levels is implied by --distributed
ign-gazebo -v 4 -z 100000000 --distributed -f $DIR/primary.sdf
