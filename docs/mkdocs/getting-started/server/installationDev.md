# Development Environment Installation

This page provides an overview on how to install the SEEREP development environment.

## VS-Code Development Container

- The VS-Code Development Container is the **easiest and recommended way** to develop
  SEEREP.

### Requirements

- Current Version of VS-Code
- Docker >= 17.12.0

### Development Container Setup

1. Clone the SEEREP repository from
   [Github](https://github.com/agri-gaia/seerep) and open it in VS-Code.

      ```bash
      git clone https://github.com/agri-gaia/seerep
      cd seerep/
      code .
      ```

2. Create a sibling folder next to the repo called `seerep-data`. This folder
   will be mounted for the data exchange between host and container. **Without
   this folder, the following steps will fail!**

      ```bash
      mkdir ../seerep-data
      ```

3. Install the [Remote
   Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
   and
   [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
   VS-Code extension with the following commands or via the extensions tab in Vs-Code.

      ```bash
      code --install-extension ms-vscode-remote.remote-containers
      code --install-extension ms-azuretools.vscode-docker
      ```

4. Press `F1` or `CTRL + SHIFT + P` in VS-Code and enter `Remote-Containers:
   Reopen Folder in Container`. **The installation process can take a couple of
   minutes** since, the docker image of SEEREP is downloaded and started.
   Additionally, all necessary VS Code extensions are installed inside the
   container and Intellisense, [pre-commit](#pre-commit-checks) hooks are
   set up.VS-Code may ask you to login to GitHub, to get the latest
   updates from the repository.

### Credentials

The default username and password for the Docker container are:

- user:`docker`
- password: `docker`.

### Pre Commit Checks

This repository uses pre-commit checks to identify simple issues in the code
base. The checks are automatically run before each commit. If you want to run
the pre-commit checks during the development of a commit, use `pre-commit run
-a`.

### Hints To Fix Errors

If the setup or the `Remote-Containers: Reopen Folder in Container` fails, here
are a couple of hints on how to fix them.

1. First make sure that the Docker container is not already running, use `docker
   container stop $VSC_SEEREP_CONTAINER_ID`, the container ID can be found using
   `docker ps`.

2. Additionally, you can delete all the data regarding SEEREP, to get a fresh
   installation:

      ```bash
      docker volume rm seerep-vscode-extensions
      docker volume rm vscode
      docker rmi ghcr.io/agri-gaia/seerep_base:latest
      (docker rmi ghcr.io/agri-gaia/seerep_server:latest)
      docker rmi vsc-seerep-*
      ```

## Manual Installation

**It is not recommended to install the following dependencies globally. Some of
them are really hard to uninstall.** If you still want to install SEEREP in this
way, follow the steps:

1. Install [ROS Noetic](https://wiki.ros.org/noetic) with the official
   [documentation](http://wiki.ros.org/noetic/Installation)
2. Install SEEREPs dependencies:
   [gRPC](https://grpc.io/docs/what-is-grpc/introduction/), [Protocol
   Buffers](https://developers.google.com/protocol-buffers),
   [Flatbuffers](https://google.github.io/flatbuffers/),
   [HighFive](https://github.com/BlueBrain/HighFive).
   Therefore, please follow the steps in the [base
   Dockerfile](https://github.com/agri-gaia/seerep/blob/main/docker/base/Dockerfile).

In order to build SEEREP, we recommend the common build tool from ROS,
[catkin](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html).
Follow the next steps to download and build seerep globally on your system.

```bash
source /opt/ros/noetic/setup.bash
mkdir -p seerep_ws/src
cd seerep_ws/src
git clone https://github.com/agri-gaia/seerep.git
cd ..
catkin build
```
