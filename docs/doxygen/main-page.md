# Home

![unstable](https://img.shields.io/badge/stability-unstable-orange)
[![License](https://img.shields.io/badge/license-BSD_3-brightgreen)](./LICENSE)
[![catkin build workflow](https://github.com/agri-gaia/seerep/actions/workflows/main.yml/badge.svg)](https://github.com/agri-gaia/seerep/actions)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](./.pre-commit-config.yaml)
[![Docker](https://img.shields.io/badge/Docker-enabled-blue?logo=docker)](./docker)

## General

The large amount of temporally and spatially high resolution sensor data of autonomous mobile robots that have to be
collected in today’s systems require a structured and, above all, efficient management and storage already during the
robot mission. We present SEEREP: A _Spatio-TemporalSemantic Environment Representation for Autonomous Mobile Robots_.
SEEREP deals with spatial, temporal and semantic linked data at once and provides an efficient query interface for all
three modalities that can be combined for high-level analyses. It supports the most popular robotic sensor data such as
images and point clouds, as well as sensor and robot coordinate frames changing over time. Furthermore, SEEREP provides
an efficient HDF5-based storage system running on the robot during operation compatible with ROS and the corresponding
sensor message definitions. The compressed HDF5 data backend can be transferred efficiently to an application server
with a running SEEREP query server providing gRPC interfaces with Protobuf and Flattbuffer message types. Partially
unstructured environments that changes over time, as for example agricultural environments, can be understood based on
high-level planning and reasoning systems using the SEEREP query server.

## Documentation

The general MkDocs based documentation of this project is available
[here](https://agri-gaia.github.io/seerep/mkdocs/home/index.html). There the general architecture is described, installation
instructions and basic tutorials are given. **Doxygen** provides the code documentation here.
