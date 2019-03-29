#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export IGN_GAZEBO_NETWORK_ROLE="SECONDARY"

# --levels is implied by --distributed
ign-gazebo-server -v 4 -z 100000000 --distributed -f $DIR/secondary.sdf
