#!/bin/sh -l

set -x

# Needed on Focal to get dart6-data for tests
apt -y install software-properties-common
apt-add-repository ppa:dartsim/ppa
apt -y install dart6-data
