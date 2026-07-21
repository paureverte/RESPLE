#!/bin/bash

USER_ID=$(id -u)
GROUP_ID=$(id -g)

NO_CACHE=""
if [[ "$1" == "--no-cache" ]]; then
    NO_CACHE="--no-cache"
fi

echo "Building resple:jazzy image for user $USER_ID..."

DOCKER_BUILDKIT=1 docker build -t resple:jazzy \
    --build-arg USER_ID=$USER_ID \
    --build-arg GROUP_ID=$GROUP_ID \
    $NO_CACHE .

echo "Built resple:jazzy. You can now run the container with: docker compose up -d"