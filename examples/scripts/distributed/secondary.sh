#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export IGN_GAZEBO_NETWORK_ROLE="SECONDARY"

ign-gazebo-server -v 4 --levels --distributed -f $DIR/../../../test/worlds/distributed/20_3_12/distsim_s.sdf
