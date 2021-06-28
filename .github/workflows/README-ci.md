# GitHub Action

The GitHub action defined in [main.yml](main.yml) builds the Docker image for the devcontainer whenever the
[Dockerfile](../../.devcontainer/Dockerfile) or the installed [dependencies](../../.devcontainer/installDependencies.sh)
change. The image is pushed to the [Docker-Hub](https://hub.docker.com/r/seerep/dev-image/) afterwards.

All pushes and pull request to the master branch are build in the docker container. Also the pre-commit checks are
performed.
