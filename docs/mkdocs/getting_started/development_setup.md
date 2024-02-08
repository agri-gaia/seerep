# Development Setup

For the development of SEEREP, we use a [Visual Studio Code Dev Container](https://code.visualstudio.com/docs
/devcontainers/containers), enabling us to utilize a container as our development environment. This approach abstracts
the operating system and automatically manages dependencies in a unified manner. For the system requirements, kindly refer
to [this section](https://code.visualstudio.com/docs/devcontainers/containers#_system-requirements).

## Setup

Initially, clone the repository with:

=== "ssh"

    ``` bash
    git clone git@github.com:agri-gaia/seerep.git
    ```

=== "https"

    ``` bash
    git clone https://github.com/agri-gaia/seerep
    ```

For the development, it is assumed that the directory designated for SEEREP's data storage is a sibling folder named
`seerep-data`, positioned adjacent to the repository. **Ensure it exists!**

!!! warning

    As a docker bind mount is used, providing an incorrect or non-existent path for the host directory will lead
    to an obscure error message upon initiating the container.

Subsequently, install the
[Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension:

<!-- markdownlint-disable-next-line MD046 -->
```bash
code --install-extension ms-vscode-remote.remote-containers
```

Launch VS Code the repository:

<!-- markdownlint-disable-next-line MD046 -->
```bash
code .
```

The presence of a `.devcontainer` folder should trigger the automatic detection of the Dev-Container environment.
It will prompt whether to reopen the repository in the container, this interaction will be displayed in the
lower right-hand corner.

<figure markdown>
  ![Vs Code reopen in container notice](../imgs/vs_code_reopen_in_container.png)
  <figcaption> Reopen in Dev-Container menu </figcaption>
</figure>

In case the window doesn't open, use either ++f1++ or ++ctrl+shift+p++ and input
`Remote-Containers: Reopen Folder in Container`. Confirm by pressing ++enter++.

Please be patient during the installation process as it involves downloading the Docker image (approximately 5 GB) and
initializing it. Additionally, the setup includes VS Code extensions, Intellisense, and pre-commit checks within the container.

## Credentials

The default username and password are: `docker`.

## Pre Commit Checks

The repository utilizes pre-commit checks to verify compliance with established coding guidelines and,
if necessary, to format the code. These checks are automatically performed before each commit.
To manually run the checks during development, use: `pre-commit run -a`.
