# SEEREP

![catkin build
workflow](https://github.com/agri-gaia/seerep/actions/workflows/main.yml/badge.svg)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)
![Architecture](https://img.shields.io/badge/Architecture-x86-blue)

## Table of Contents

- [General](#general)
- [Why](#why)
- [Overview of SEEREP](#overview-of-seerep)
- [Documentation](#further-documentation)

## General

SEEREP aims at improving the understanding of unstructured environments for
robotic systems. For that, SEEREP provides a **SE**mantic **E**nvironment
**REP**resentation for planning and reasoning algorithms. The main features of
SEEREP include:

- Fast spatio-temporal-semantic queries with
  [gRPC](https://grpc.io/docs/what-is-grpc/introduction/).
- Storeage of data generated by the robotic system.
  - Offline on the robot, e.g. no or slow internet connection.
  - Save data on a server-cluster with
     [gRPC](https://grpc.io/docs/what-is-grpc/introduction/)
- Switch between [Protocol Buffers
  (PB)](https://developers.google.com/protocol-buffers/docs/overview) /
  [Flatbuffers (FB)](https://google.github.io/flatbuffers/) as the messaging
  format.

## Why

Most present environment representations focus on one or two domains, e.g.
spatial-temporal. But in order to enable a robot to fully understand its
environment on a higher level, a spatio-temporal-semantic representation is
needed. Semantic entries are especially useful to disambiguate sensor data based
on the context.

## Overview of SEEREP

The following graphic provides a high level overview of SEEREP and its
above-mentioned features.

![](docs/imgs//SEEREP-Overview.svg)

## Documentation

The documentation consists of two parts, [MkDocs](#mkdocs) which focuses on
concepts, tutorials and general information, while [Doxygen](#doxygen) is used
for code documentation.

### MkDocs

Since the documentation is currently not published to GitHub Pages, it needs to
be run locally. For that, switch into the folder where the `mkdocs.yml` is
located and use `mkdocs serve` to start the server. The page should then be
available under [http://127.0.0.1:8000/](http://127.0.0.1:8000/).

### Doxygen

Same situation as with [MkDocs](#mkdocs), currently Doxygen is only available
locally. To create the Doxygen output switch into the folder where the
`Doxyfile` is located and run `doxygen Doxyfile`. Now an `html/` folder should
be in the same directory.

If you are **not working in the devcontainer** you can simply open the
`index.html` with your browser of choice. **Otherwise,** switch into the `html`
folder and use `python3 -m http.server` to start a local web server to serve the
content. The page should be available under the default
[http://0.0.0.0:8000/](http://0.0.0.0:8000/) address.

If you want to run [MkDocs](#mkdocs) and [Doxygen](#doxygen) at the same time
you need to provide a different port to the  Doxygen web server, use `python3 -m
http.server 8002` instead.
