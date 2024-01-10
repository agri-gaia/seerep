# Server Deployment

SEEREP is distributed as a Docker image and can be obtained using the following command:

```bash
docker pull ghcr.io/agri-gaia/seerep_server:[tag/latest]
```

A list of stable tags is accessible [here](https://github.com/agri-gaia/seerep/releases). The latest tag can be
employed to acquire the most recent updates, but it may result in potential instabilities.

## Storage

SEEREP is recommended to be utilized with a `named volume mount` for persisting/reading data to/from HDF5.
When a volume is mounted into a container, the storage path of the volume is incorporated into the container.
This enables the usage of existing data from the host system or the transfer of new HDF5 files to SERREP with tools
such as [rsync](https://linux.die.net/man/1/rsync).

<!-- markdownlint-disable -->
!!! Warning

    In the absence of the storage path for a volume on the host system, Docker will attempt to create it.
<!-- markdownlint-enable -->

## Configuration

SEEREP offers configuration options through CLI arguments, a configuration file, or environment variables.
The table below provides an overview of the parameters and their respective purposes.
Additional details are available on the [reference page](../reference/server_configuration.md).

| Parameter     | Description                                  |
| ------------- | -------------------------------------------- |
| `data-folder` | Path of the mounted volume for storage       |
| `port`        | Port of the gRPC server                      |
| `log-path`    | Path to store log files                      |
| `log-level`   | Filter log output based on levels            |

## Docker Run

For a quick start of the server using `docker run`, use:

```bash
docker run \
  --volume=/your/local/absolute/path:/mnt/seerep-data \
  --publish=9090:9090 \
  --name=seerep_server \
  --tty \
  ghcr.io/agri-gaia/seerep_server:[tag/latest] \
  --data-folder=/mnt/seerep-data \
  --log-path=/mnt/seerep-data/log \
  --log-level=info
```

## Docker-Compose

To provide a more specific configuration for the server, utilize `docker compose`:

```yaml title="docker-compose.yml"
version: "3.6"
services:
  seerep:
    image: ghcr.io/agri-gaia/seerep_server:[tag/latest]
    tty: true
    container_name: seerep_server
    ports:
      - 9090:9090
    volumes:
      - /your/local/absolute/path:/mnt/seerep-data
    environment:
      - TZ=Europe/Berlin
      - SEEREP_DATA_FOLDER=/mnt/seerep-data/
      - SEEREP_LOG_PATH=/mnt/seerep-data/log
      - SEEREP_LOG_LEVEL=info
volumes:
  seerep-data:
```
