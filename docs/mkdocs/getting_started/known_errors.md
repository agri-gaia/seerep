# Known Errors

This page serves as a list of common errors encountered while developing the project.
Look at it when you're facing problems, maybe it's listed here, and if not, consider adding it.

## Downloading the Docker Image

When attempting to download the Docker image from GitHub Container Registry (`ghcr.io`), make sure to
[authenticate with a personal access token](https://docs.github.com/en/packages/
working-with-a-github-packages-registry
/working-with-the-container-registry#authenticating-with-a-personal-access-token-classic). Unfortunately GitHub
does not allow downloading packages otherwise.

<figure markdown>
  ![Vs Code reopen in container notice](../imgs/ghcr.io_login_error.png){width="600"}
  <figcaption> ghcr.io login error </figcaption>
</figure>

## Starting the Dev-Container

If the setup or the `Remote-Containers: Reopen Folder in Container` fails, one of the following steps may resolve it:

1. Make sure that the container is not running in the backgound:

    ```shell
    docker stop $(docker ps | grep seerep | awk '{print $1}')
    ```

2. Sometimes a fresh install fixes everything, you can use the following commands to delete everything regarding
  the Dev-Container from Docker:

    ```shell
    docker rm $(docker container ls -a | grep seerep | awk '{print $1}')
    docker rmi $(docker image ls -a | grep seerep | awk '{print $1}')
    docker volume rm seerep-vscode-extensions vscode
    ```

## Running out of RAM when compiling

Compiling the server executable (especially when using WSL[^1]) may require more than 13 GB of RAM.
For additional information, see the documentation
[here](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#configuring-memory-use).

```shell
pip install psutil
catkin build --mem-limit 40% / 8G
```

[^1]: Windows Subsystem for Linux: A way to access Linux on a Windows system without using a VM or a dualboot setup.
