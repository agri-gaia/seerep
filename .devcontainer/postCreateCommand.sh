#! /bin/bash

#copy vs code config to root
cp -r /seerep/src/.devcontainer/.vscode/ /seerep/.vscode/

#build the workspace with catkin
catkin config  --workspace /seerep --cmake-args -Wno-deprecated -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK"
catkin build --workspace /seerep

#install pre-commit in the git repo
cd /seerep/src || exit 1
pre-commit install --install-hooks
