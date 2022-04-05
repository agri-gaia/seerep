#!/bin/bash
# shellcheck disable=SC1091

echo "arguments"
echo "$@"

source /opt/ros/noetic/setup.bash

source /home/docker/seerep_ws/devel/setup.bash

/home/docker/seerep_ws/devel/bin/seerep-server_server "$@"
