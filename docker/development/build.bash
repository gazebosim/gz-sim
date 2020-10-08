#!/usr/bin/env bash

if [ $# -ne 2 ]
then
  echo "Usage: $0 image-name path-to-packages.apt"
  exit 1
fi

# build context needs to be the path to the repo's .github/ci/ directory
docker build -t $1 --build-arg user_id=$(id -u) -f ./Dockerfile $2
