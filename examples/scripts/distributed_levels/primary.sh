#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export IGN_GAZEBO_NETWORK_ROLE="PRIMARY"
export IGN_GAZEBO_NETWORK_SECONDARIES=2

ign-gazebo -v 4 --distributed --levels -f $DIR/primary.sdf
