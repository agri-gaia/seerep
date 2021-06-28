# seerep

<a name="top"></a>

![catkin build workflow](https://github.com/agri-gaia/seerep/actions/workflows/main.yml/badge.svg)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)

## Table of Contents

- [General](#general)
- [Quick Start](#quickstart)
- [Manual Installation](#installation)
- [pre-commit formatting checks](#precommit)

<a name="general"></a>
<p align="right"><a href="#top">Top</a></p>

## General

seerep stands for SEmantic Environment REPresentation and represents a system that efficiently implements communication
and storage layers for sensor and environmental data.

In the current version, ROS (Robot Operating System) messages of configured topics are converted to Protobuf and
transmitted to a server using grpc. The server stores the received data in an HDF5 file. The data can later be accessed
efficiently enabling goal-oriented access to specific datasets and data ranges.

![](docs/workflow.svg)

<a name="quickstart"></a>
<p align="right"><a href="#top">Top</a></p>

## Quick Start

### VS Code devcontainer

1. Clone this repo and open it in VS Code.
2. Install the extensions `ms-vscode-remote.remote-containers` and `ms-azuretools.vscode-docker`.
3. Press `F1` or `CTRL + SHIFT + P` and enter `Remote-Containers: Reopen Folder in Container`
4. This creates a docker container based on
   [seerep/dev-image](https://hub.docker.com/repository/docker/seerep/dev-image), installs all necessary VS Code
   extensions, builds the workspace a first time, sets up Intellisense, installs the [pre-commit](#precommit) hooks and
   opens it in VS Code.

   There current folder will not be mounted in the docker container! During the setup a persistent Docker Volume gets
   created and the workspace of the container is saved in this persistent volume. Therefore, Code changes in the
   devcontainer will not appear directly in the cloned repo from step 1. With the persistent volume the devcontainer can
   be easily used on remote host. For more information see [.devcontainer/README-devcontainer.md](.devcontainer/README-devcontainer.md)

   Use `pre-commit run -a` in the workspace folder to check the code style before commiting. In the Docker image the
   pre-commit checks are installed in the git so a commit is just possible if the checks succeed.

5. default user: `docker` with password: `docker`

<a name="installation"></a>
<p align="right"><a href="#top">Top</a></p>

## Manual Installation

If you prefer a native installation without docker take a look at
[docs/manualInstallation.md](docs/manualInstallation.md).

<a name="precommit"></a>
<p align="right"><a href="#top">Top</a></p>

## pre-commit formatting checks

This repo has a [pre-commit](https://pre-commit.com/) check that runs in CI. You can use this locally and set it up to
run automatically before you commit something. In the devcontainer it is already pre-installed. To install, use pip:

```bash
pip3 install --user pre-commit
```

To run over all the files in the repo manually:

```bash
pre-commit run -a
```

To run pre-commit automatically before committing in the local repo, install the git hooks:

```bash
pre-commit install
```
