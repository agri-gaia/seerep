#!/bin/bash
# shellcheck disable=SC1091

source /opt/ros/noetic/setup.bash
source /seerep/devel/setup.bash

/seerep/devel/bin/seerep_server "$@"
