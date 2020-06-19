#!/bin/bash

echo "Launching horizon_highway_slam:latest"

SHARED_DOCKER_DIR=/root/shared_dir
SHARED_HOST_DIR=$HOME/shared_dir

VOLUMES="--volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw"

mkdir -p $SHARED_HOST_DIR

docker run \
    -it --rm \
    $VOLUMES \
    --net=host \
    horizon_highway_slam:latest

