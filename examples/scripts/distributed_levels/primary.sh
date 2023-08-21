#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# --levels is implied by --network-role
gz sim -v 4 -z 100000000 --network-role primary --network-secondaries 2 $DIR/primary.sdf
