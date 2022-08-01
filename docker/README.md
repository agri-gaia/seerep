# Seerep Images

The seerep-image is divided into two parts: base and server.

All images are stored in the [github-registry](https://github.com/orgs/agri-gaia/packages?repo_name=seerep)

## Base Image

The base image itself is derived from [ros:neotic](http://wiki.ros.org/noetic) and installs all neccessary dependencies
for ros and seerep. This image is also used for the vs-code dev-container.

- source: <https://github.com/agri-gaia/seerep/pkgs/container/seerep_base>
- folder: ./base

## Server Container

The Server Container can be used to either start seerep in a Kubernetes Cluster or locally.

For local development, one can use the docker-compose.yaml to simple start the server. Within the [docker-compose.yaml](docker-compose.yml)
one can change the *command:* in order to change the directory used by the server for created files.

The deployment of seerep on a Kubernetes Cluster can be achieved by "applying" the [seerep-development.yaml](seerep-development.yaml).
This will deploy the server, a service to access the server and an ingress, to reach the server through a nice url.
As for now, the server will be deployed into the namespace *robot*, which means that it needs to be created first.
But the namespaces can simply be changed.

Before deploying seerep to any cluster, one needs to create two secrets:

1. containing a certificate
2. access credentials for the container registry on git.ni.de

- source: git.ni.dfki.de:5050/agrigaia/seerep/dev-server
- folder: /dev-server
