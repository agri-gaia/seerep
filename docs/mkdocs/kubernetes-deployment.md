# Kubernetes Deployment

Besides, the local installation and the usage of an available docker container (see [installation](installation.md)),
one can also deploy the seerep-server within a kubernetes cluster.

## Relevant files

Seerep can either be installed with the latest development state or the latest stable version. The relevant files can
be found under

* /docker/kustomize/base --> development
* /docker/kustomize/overlays/production --> latest stable release

The base-folder contains all yaml-files for a cluster deployments. This includes

* Deplyoment
* PersistentVolume and PersistentVolumeClaim
* Service
* Ingress
* Configuration

The yaml-file for the cluster is create via [Kustomize](https://kubectl.docs.kubernetes.io/references/kustomize/), hence
this folder also contains a kustomization.yaml which puts everything together.

The /overlay/production folder contains a second kustomization.yaml. Within this file everything needed to install a
production system is overridden. This means, that the sealed secret is replaced with a new one, while the base secret
is deleted. Further, the labels and the names of PV and PVC are changed to create new storage explicitly for the
production system. Finally, the used image is replaced with the latest stable release.

The usage of an overlay, thereby, follows the principles of Kustomize.

## Building with Kustomize

Before one can build the kubernetes manifest, one needs to install Kustomize ([Kustomize installation](https://kubectl.docs.kubernetes.io/installation/kustomize/))
The easiest way to that is the download the latest binary from the [offical release page](https://github.com/kubernetes-sigs/kustomize/releases)

To install the base-version of seerep either one can run:

```
kustomize build base/ > seerep-deployment.yaml
```

to store the manifest in a separate yaml file. Or directly use [kubectl](https://kubernetes.io/docs/reference/kubectl/):

```
kubectl apply -k base/
```

In order to install the production version, the commands look slightly different:

```
kustomize build overlays/production/ > seerep-deployment.yaml
```

```
kubectl apply -k overlays/production/
```

If someone has a running [ArgoCD](https://argo-cd.readthedocs.io/en/stable/) instance, it is also
possible to integrate seerep as a project into ArgoCD.

## Sealed Secrets

The certificates used for the secured ingress are created as [sealed-secret](https://github.com/bitnami-labs/sealed-secrets).
Hence, the secret can safely be stored in a repository. The sealed secret controller installed
within the cluster will take care of unsealing the secret and make it usable. To combine Kustomize and
sealed secrets this blogs-post was followed [faun.pub](https://faun.pub/sealing-secrets-with-kustomize-51d1b79105d8)
