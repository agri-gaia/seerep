#! /bin/bash

#copy vs code config to root
cp -r /home/docker/workspace/src/.devcontainer/.vscode /home/docker/workspace/.vscode

#build the workspace with catkin
catkin config  --workspace /home/docker/workspace --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build --workspace /home/docker/workspace

#install pre-commit in the git repo
cd /home/docker/workspace/src || exit 1
pre-commit install
