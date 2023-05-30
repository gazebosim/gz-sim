#!/bin/bash
# Command line parameters:
# 1 - github organization name. For example gazebosim or osrf.
# 2 - the name of the Gazebo repository. For example gz-math.
# 3 - the name of the branch. For example gz-math7

set -o errexit
set -o verbose

# todo(nkoenig) Update the database to handle templated names.
# todo(nkoenig) Use application tokens instead of jwt, which can expire.
curl -k -X POST -d @$1 https://api.gazebosim.org/1.0/benchmarks/gazebo --header 'authorization: Bearer '$AUTH0_JWT_TOKEN
