#!/bin/bash

# --- CONFIGURATION ---
# Path to your package in WSL
WSL_DIR="$HOME/catkin_ws/src/hakuroukun_boustrophedon_with_cones"
# Destination path in the Docker container
DOCKER_DIR="/root/catkin_ws/src/hakuroukun_boustrophedon_with_cones"
# Name of your Docker container
CONTAINER_NAME="hakuroukun-robot"
# ---------------------

echo "Watching $WSL_DIR for changes..."
echo "Any file save in WSL will automatically sync to $CONTAINER_NAME."

# Monitor the folder recursively for modifications, creations, or deletions
while inotifywait -r -e modify,create,delete,move "$WSL_DIR"; do
    echo "Change detected! Syncing to Docker container..."
    docker cp "$WSL_DIR/." "$CONTAINER_NAME:$DOCKER_DIR"
    echo "Sync complete."
    echo "------------------------------------------------"
done