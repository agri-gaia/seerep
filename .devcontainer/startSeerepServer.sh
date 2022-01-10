#! /bin/bash

source /opt/ros/noetic/setup.bash

# get the directory this script is saved in
DIR="$( cd "$( dirname "$0" )" && pwd )"

#build everything
catkin build --workspace /home/docker/workspace  || exit

#run the server

# the next line is needed to avoid https://github.com/koalaman/shellcheck/wiki/SC1091 if catkin build hasn't run yet
# shellcheck source=/dev/null
source /home/docker/workspace/devel/setup.bash
"$DIR"/../../devel/bin/seerep-server_server /home/docker/workspace/seerep-data || exit
