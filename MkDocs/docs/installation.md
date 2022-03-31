# Installation

This page provides an overview on ways to install SEEREP.

## VS Code Devcontainer

!!! tip "Recommended way of installing SEEREP"

**Requirement**:

Docker Version >= 17.12.0

1. Clone the SEEREP repository from
   [Github](https://github.com/agri-gaia/seerep) and open it in VS-Code.

      ```
      git clone https://github.com/agri-gaia/seerep
      cd seerep/ && code
      ```

2. Create a sibling folder to the repo folder called `seerep-data`. This folder
   will be mounted for the data exchange between host and container. **Without
   this folder, the following steps will fail!**

      ```
      mkdir ../seerep-data
      ```

3. Install the _Remote Containers_ and _Docker_ extension.

      ```
      code --install-extension ms-vscode-remote.remote-containers
      code --install-extension ms-azuretools.vscode-docker
      ```

4. Press `F1` or `CTRL + SHIFT + P` in VS-Code and enter `Remote-Containers:
   Reopen Folder in Container
      - **This can take a couple of minutes** since, a docker container based on
         the [seerep/dev-image](https://hub.docker.com/r/seerep/dev-image) is
         downloaded and started. Additionally, all necessary VS Code extensions
         are installed and Intellisense / [pre-commit](#pre-commit-checks) hooks
         are set up.
      - VS-Code may ask you to login, to get the latest updates from the
        repository.

The current folder will be mounted in the docker container with default user:
`docker` and password: `docker`.

### Pre Commit Checks

   Use `pre-commit run -a` in the workspace folder to check the code style
   before committing. The docker image already has the pre-commit checks
   installed, thus committing is only possible if the checks succeed.

### Remote Host

!!! warning  "Currently not tested !"

If you want to use a
[devcontainer](https://code.visualstudio.com/docs/remote/containers) on a remote
host, add the following to `settings.json` of your VS-Code config in your local
workspace and replace the placeholders with your values:

```
"docker.host": "ssh://your-remote-user@your-remote-machine-fqdn-or-ip-here",
```

Also update the workspace mount to a volume in the `devcontainer.json` file:

```
"workspaceMount": "source=seerep-ws,target=/home/docker/workspace/src,type=volume",
```

Make sure that your SSH agent is running and that it knows the keys to connect
to the remote host. Instructions on that can be found
[here](https://code.visualstudio.com/docs/remote/containers#_using-ssh-keys).

## Manual Installation

1. Install ROS Noetic with the offical
   [documentation](http://wiki.ros.org/noetic/Installation)
2. Install SEEREPs dependencies:
      - [gRPC](https://grpc.io/docs/what-is-grpc/introduction/)
      - [Protocol Buffers](https://developers.google.com/protocol-buffers)
      - [Flatbuffers](https://google.github.io/flatbuffers/)
      - [HighFive](https://github.com/BlueBrain/HighFive)

Therefore, please follow the
[installDependecies.sh](https://github.com/agri-gaia/seerep/blob/master/.devcontainer/installDependencies.sh)
script.

!!! warning
    It is not recommended to install these dependencies globally. Some of them are really hard to uninstall !!

### Building SEEREP

We provide two ways to build SEEREP: First with [catkin
build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)
and second with cmake/make.

#### Catkin Build

```
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p seerep_ws/src
cd seerep_ws/src
git clone git@github.com:agri-gaia/seerep.git
cd ..
catkin build
```
