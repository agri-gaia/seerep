# Known Issues

This page serves as a list of fixes or workarounds for common errors encountered while developing SEEREP.
If you find a fix that isn't included, please consider adding it.

## Starting the Dev-Container Fails

If the automatic contianer setup or the `Remote-Containers: Reopen Folder in Container` fails,
one of the following steps may resolve it:

1. Make sure that the container is not running in the backgound:

    ```shell
    docker stop $(docker ps | grep seerep | awk '{print $1}')
    ```

2. Often, a fresh install can fix problems. Use the following commands to delete everything regarding SEEREP from Docker:

    ```shell
    # remove container
    docker rm $(docker container ls -a | grep seerep | awk '{print $1}')
    # remove images
    docker rmi $(docker image ls -a | grep seerep | awk '{print $1}')
    # remove volumes
    docker volume rm seerep-vscode-extensions vscode
    ```

## Running out of RAM when Compiling

When compiling and linking the server executable (especially when using [WSL](https://ubuntu.com/desktop/wsl)) catkin may
allocate more than 13 GB of system memory. To restrict the memory usage of `catkin` use:

```shell
pip3 install psutil
# relative to the full system memory
catkin build --mem-limit 40%
# absolute maximum of system memory to use
catkin build --mem-limit 8G
```

For more details, refer to the
[catkin documentation](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#configuring-memory-use).
