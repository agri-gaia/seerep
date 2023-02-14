# Local Deployment

The local deployment is based on the [seerep_server docker image](https://github.com/agri-gaia/seerep/pkgs/container/seerep_server).
The image with the latest (unstable) version can be pulled with the following command. It is recommended to use a version
tag instead of `latest`.

```bash
docker pull ghcr.io/agri-gaia/seerep_server:latest
```

## docker run

Run the following command to start the server using `docker run`. It is recommended to use a version
tag instead of `latest`.

```bash
docker run \
  --volume=seerep-data:/mnt/seerep-data \
  --publish=9090:9090 \
  --name=seerep_server \
  --tty \
  ghcr.io/agri-gaia/seerep_server:latest \
  --data-folder=/mnt/seerep-data
```

## docker-compose

Run `docker-compose up` in the folder of the docker-compose.yml to start the server. It is recommended to use a version
tag instead of `latest`.

For this [docker compose](https://docs.docker.com/compose/) has to be installed. In the latest version `docker compose`
without a hyphen as part of the Docker CLI replaces `docker-compose`.

Example docker-compose.yml:

```docker
version: "3.6"
services:
  seerep:
    image: ghcr.io/agri-gaia/seerep_server:latest
    tty: true
    container_name: seerep_server
    ports:
      # the gRPC port
      - 9090:9090
    volumes:
      # persist the data folder
      - seerep-data:/mnt/seerep-data #using docker volume
      #- /your/local/absolute/path:/mnt/seerep-data #using host folder
    environment:
      - TZ=Europe/Berlin
      - SEEREP_DATA_FOLDER="/mnt/seerep-data"
      - SEEREP_LOG_PATH="/mnt/seerep-data/log"
      - SEEREP_LOG_LEVEL="info"
volumes:
  seerep-data:
```
