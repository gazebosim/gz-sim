#!/usr/bin/env bash
#
# Purpose
#   This script is designed to build a docker image of a Gazebo distribution.
#   See README.md and run.bash for more information.

if [ $# -eq 0 ]
then
    echo "Usage: $0 <Gazebo meta-package name> <dockerfile>"
    echo "Example: $0 gz-garden ./Dockerfile.gz"
    exit 1
fi

user_id=$(id -u)
image_name=$1
dir_name=$(dirname $2)
image_plus_tag=$image_name:$(date +%Y_%m_%d_%H%M)

docker build --rm -t $image_plus_tag --build-arg user_id=$user_id --build-arg gz_distribution=$1 -f $2 .
docker tag $image_plus_tag $image_name:latest

echo "Built $image_plus_tag and tagged as $image_name:latest"
