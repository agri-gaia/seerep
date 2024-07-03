# Documentation Setup

The documentation is divided into two main parts:
[MkDocs](https://agri-gaia.github.io/seerep/mkdocs/home/index.html) is used for general documentation
and [Doxygen](https://agri-gaia.github.io/seerep/doxygen/index.html) is used for C++ code documentation.

## Dependencies

Installing the dependencies in the following sections is only ncessaary if you are **not** using the
VS-Code Development Container.

## MkDocs

To install the Python dependencies, navigate to the repository's main folder and run:

```bash
pip3 install -r docker/base/requirements.docs.txt
```

To build and deploy MkDocs locally use:

```bash
# ensure that the mkdocs.yaml is located in the directory
mkdocs serve
```

The page should then be available under [http://127.0.0.1:8000/](http://127.0.0.1:8000/).
Live reloading is enabled, saved changes are automatically updated.

## Doxygen

To install a recent version of Doxygen, it is necessary to build it from source:

```bash
sudo apt install flex bison make \
    && export DOX_VER="Release_1_9_3" \
    && wget https://github.com/doxygen/doxygen/archive/refs/tags/${DOX_VER}.tar.gz \
    && tar -xf ${DOX_VER}.tar.gz \
    && mkdir -p doxygen-${DOX_VER}/build \
    && cd doxygen-${DOX_VER}/build \
    && cmake -G "Unix Makefiles" .. \
    && make -j"$(nproc)" \
    && sudo make install \
    && cd ../.. \
    && rm -rf doxygen-${DOX_VER} ${DOX_VER}.tar.gz \
    && unset DOX_VER
```

To generate the Doxygen documentation, run the following command in the repository's main directory:

```bash
doxygen Doxyfile
```

To start a local web server to serve the content use:

```bash
cd doxygen-output/html/
python3 -m http.server
```

The page should be accessible at the default [http://0.0.0.0:8000/](http://0.0.0.0:8000/).
If you want to run both MkDocs and Doxygen simultaneously, you need to assign a
different port to the HTTP server for Doxygen, for example by using:

```bash
python3 -m http.server 9000
```

## GitHub Pages Deployment

Both documentation pages are automatically deployed to GitHub Pages through a
[workflow](https://github.com/agri-gaia/seerep/blob/mkdocs-update-getting-started/.github/workflows/build-docs.yml).
A new version is released after a pull request to the main branch.
Publishing both documentations requires a workaround, as a repository typically only supports one URL.
The main page is a simple HTML document that links to the individual pages:

```html title="index.html"
--8<-- "docs/gh-pages/index.html"
```
