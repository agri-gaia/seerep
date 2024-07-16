# Server Deployment

## Docker

The easiest way to start a production instance of SEEREP is with `docker compose`,
using the corresponding file in the `docker/server` directory:

```bash
cd docker/server
docker compose up
```

!!! info "Use of named Docker volumes"
    <!-- markdownlint-disable-next-line -->
    Compared to the Dev-Container, the production setup uses
    [named Docker volumes](https://docs.docker.com/storage/volumes/). Which do
    not require any specific directory on the host system. The volumes also keeps
    the data between restarts.     For an empty server delete the volumes with
    `docker volume rm seerep_log seerep_data`.

Configuration of the compose setup is done through the `.env` file located in
the same directory. For details on the individual options, refer to the
[configuration](#configuration) section.

A closer look at the Docker compose file might lead to questions about the extra
service, that adjusts the owner of the log and data directories. The main reason
for this is that named volumes are created with root permissions when the mount
location does not exist. However since the container is run as non-root for
security reasons (as should ever other container), we are not able to change the
permissions without hard-coding the mount locations in the Dockerfile. To change
the permissions, a very small container mounts the volumes and changes the
permission to the specified `uid` and `guid` after which SEEREP is started with
the same user. Additional information can be found in the
[PR#376](https://github.com/agri-gaia/seerep/pull/376) discussion.

## Kubernetes

!!! warning "Not actively maintained!"
    <!-- markdownlint-disable-next-line -->
    The deployment of SEEREP using Kubernetes is currently not actively maintained.

### Relevant files

Seerep can either be installed with the latest development state or the latest
stable version. The relevant files can be found under:

```bash
/docker/kustomize/base --> development
/docker/kustomize/overlays/production --> latest stable release
```

The base-folder contains all yaml-files for a cluster deployment. This includes:

* Deployment
* PersistentVolume (PV) and PersistentVolumeClaim (PVC)
* Service
* Ingress
* Configuration

The yaml-file for the cluster is create via
[Kustomize](https://kubectl.docs.kubernetes.io/references/kustomize/), hence
this folder also contains a kustomization.yaml which puts everything together.

The `/overlay/production` directory contains a second `kustomization.yaml`.
Within this file everything needed to install a production system is overridden.
This means, that the sealed secret is replaced with a new one, while the base
secret is deleted. Further, the labels and the names of PV and PVC are changed
to create new storage explicitly for the production system. Finally, the used
image is replaced with the latest stable release.

The usage of an overlay, thereby, follows the principles of Kustomize.

### Building with Kustomize

Kustomize needs to be installed before building the Kubernetes manifest
([Kustomize installation](https://kubectl.docs.kubernetes.io/installation/kustomize/)).
The easiest way to do that is to download the latest binary from the
[offical release page](https://github.com/kubernetes-sigs/kustomize/releases).

To install the base version of SEEREP, one can either run:

```bash
kustomize build base/ > seerep-deployment.yaml
```

to store the manifest in a separate `.yaml` file. Or directly use
[kubectl](https://kubernetes.io/docs/reference/kubectl/):

```bash
kubectl apply -k base/
```

In order to install the production version, the commands look slightly different:

```bash
kustomize build overlays/production/ > seerep-deployment.yaml
```

```bash
kubectl apply -k overlays/production/
```

If a [ArgoCD](https://argo-cd.readthedocs.io/en/stable/) instance is available,
SEEREP can also be added as a project.

### Sealed Secrets

The certificates used for the secured ingress are created as a
[sealed-secret](https://github.com/bitnami-labs/sealed-secrets).
Hence, the secret can safely be stored in a repository. The sealed secret
controller installed within the cluster will take care of unsealing the secret
and make it usable. To combine Kustomize and sealed secrets this blogs-post was
followed [faun.pub](https://faun.pub/sealing-secrets-with-kustomize-51d1b79105d8).

## Configuration

SEEREP can be configured in three different ways: via command line arguments, a
config file, or environment variables.

### Command line

To get a full list of all arguments use `--help`:

```bash
Allowed options:

Generic options:
  -v [ --version ]                      Get a version string
  --help                                Help message
  -c [ --config ] arg                   Path to a configuration file

Configuration:
  -D [ --data-folder ] arg (=/seerep/src)
                                        Data storage folder
  -L [ --log-path ] arg (=/seerep/src)  Path to store the logs
  --log-level arg (=info)               log-level [trace, debug, info, warning,
                                        error, fatal]
  -p [ --port ] arg (=9090)             gRPC port to use
```

The configuration parameters are provided with default values (shown in parentheses).

### Config file

The command line options can also be set through a config file:

```bash
$(which seerep_server) --config seerep.cfg
```

An example config file is shown below:

```cfg title="seerep.cfg"
--8<-- "seerep.cfg"
```

### Environment Variables

Additionally, SEEREP can be configured using environment variables, which correspond
to the command line arguments as follows:

| Environment variable  | Command Line              |
|---                    |---                        |
| SEEREP_DATA_FOLDER    | --data-folder             |
| SEEREP_LOG_PATH       | --log-path                |
| SEEREP_LOG_LEVEL      | --log-level               |
| SEEREP_PORT           | --port                    |
