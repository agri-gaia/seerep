# Devcontainer

This folder contains all files needed for the development container:

1. Dockerfile and installDependencies.sh: Creation of the docker image
2. devcontainer.json and postCreateCommand.sh: Instructions for VS Code on how to create the dev container based on
  the previously created image
3. .vscode: The VS Code settings folder. To be used in the devcontainer to create a out of the box working Intellisense

Some information regarding devcontainers:

- <https://code.visualstudio.com/docs/remote/containers>
- <https://code.visualstudio.com/docs/remote/create-dev-container>
- <https://code.visualstudio.com/docs/remote/containers-advanced>
- <https://code.visualstudio.com/docs/remote/devcontainerjson-reference>
