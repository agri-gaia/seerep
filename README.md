#

<div align="center">
  <img alt="Redoc logo" src="docs/logo/Seerep_Logo.svg" width="300px" />

<h3> Spatio-Temporal-Semantic Sensor Data Management <br>
for Agricultural Robotics </h3>

  [![catkin build workflow](https://github.com/agri-gaia/seerep/actions/workflows/main.yml/badge.svg)](https://github.com/agri-gaia/seerep/actions)
  ![unstable](https://img.shields.io/badge/Stability-experimental-orange)
  [![License](https://img.shields.io/badge/License-BSD_3-brightgreen)](./LICENSE)

  <b><a href="https://agri-gaia.github.io/seerep/mkdocs/home/index.html">
  Documentation</a></b>
  |
  <b><a href="https://github.com/agri-gaia/seerep/tree/main/examples/python/gRPC">
  Examples</a></b>
</div>

## Table of Contents

- [Features](#-features)
- [Motivation](#muscle-motivation)
- [Getting Started](#-getting-started)
- [Maintainer](#wrench-maintainer)
- [Related Publications](#memo-related-publications)
- [License](#-license)

## 🎯 Features

- Robotic sensor data management integrated with [ROS](https://www.ros.org/).
- Creation of sub-datasets based on sub-symbolic information (positions,
timestamps) and their corresponding symbolic (semantic) information.
- Automated generation and evaluation (e.g neural network predictions) of
datasets.
- GDPR filtering of datasets (e.g excluding public roads) based on spatial
polygon queries.

## :muscle: Motivation

A key transition in robotics involves the adoption of data-driven approaches.
As a result, increasing amounts of data are being captured by high-resolution
sensors such as cameras and LiDAR.
With ROS, the typical workflow involves recording rosbags during missions and
uploading the resulting files to network or blob storage, organized according to
use-case specific hierarchies.
Later retrieval of data with specific properties, such as particular weather
conditions or a specific area of interest, is non-trivial. The relevant data
must be downloaded to open the rosbag where access to the data is only provided
through the time modality.

In contrast, SEEREP offers an integrated solution for managing robotic sensor
data and generating sub-datasets based on spatial, temporal, or semantic
queries. Simplifying the workflow to uploading the sensor data post-mission and
offering a gRPC-based query interface for subsequent applications.

## 🚀 Getting Started

The simplest way to start SEEREP is by using `docker compose` with the
configuration provided in the `docker/server` directory:

```bash
git clone https://github.com/agri-gaia/seerep.git
cd seerep/docker/server
docker compose up
```

Which should produce an output like this:

```bash
seerep_server | [2024-07-11 13:40:00.853800]<info>: Initialized logging
seerep_server | [2024-07-11 13:40:00.860308]<info>: The used logging folder is: /mnt/seerep_data/log/
seerep_server | [2024-07-11 13:40:00.860730]<info>: Current timezone: CET
seerep_server | [2024-07-11 13:40:00.861233]<info>: SEEREP version: N/A
seerep_server | [2024-07-11 13:40:00.861567]<info>: The used data folder is: /mnt/seerep_data/
seerep_server | [2024-07-11 13:40:00.865541]<info>: Addded Protocol Buffers gRPC-services
seerep_server | [2024-07-11 13:40:00.867241]<info>: Added Flatbuffers gRPC services
seerep_server | [2024-07-11 13:40:00.903333]<info>: Serving gRPC Server on "[::]:9090"
```

For other ways to deploy SEEREP, check the
[documentation](https://agri-gaia.github.io/seerep/mkdocs/getting-started/server_deployment/).

Refer to the [examples section](https://github.com/agri-gaia/seerep/tree/update-readme/examples/python/gRPC)
for instructions on uploading data.

## :wrench: Maintainer

[Mark Niemeyer](https://github.com/Mark-Niemeyer) <br>
[mark.niemeyer@dfki.de](mailto:mark.niemeyer@dfki.de)

[German Research Center for Artificial Intelligence](https://www.dfki.de/en/web)<br>
[DFKI Niedersachsen](https://www.dfki.de/en/web/about-us/locations-contact/osnabrueck-oldenburg)<br>
[Plan-Based Robot Control](https://www.dfki.de/en/web/research/research-departments/plan-based-robot-control)<br>

## :memo: Related Publications

```bibtex
@inproceedings{Niemeyer2024,
  author = {Niemeyer, Mark and Arkenau, Julian and Pütz, Sebastian and
  Hertzberg, Joachim},
  title = {Streamlined Acquisition of Large Sensor Data for Autonomous Mobile
  Robots to   Enable Efficient Creation and Analysis of Datasets },
  booktitle = {2024 IEEE International Conference on Robotics and Automation (ICRA)},
  year = {2024},
  publisher = {IEEE}
}

@inproceedings{Niemeyer2023,
  author = {Niemeyer, Mark and Renz, Marian and Hertzberg, Joachim},
  title = {Object Anchoring for Autonomous Robots using the Spatio-Temporal-Semantic
  Environment Representation SEEREP},
  booktitle = {KI 2023. German Conference on Artificial Intelligence (KI-2023)},
  year = {2023},
  publisher = {Springer}
}

@inproceedings{Niemeyer2022,
  author = {Niemeyer, Mark and Pütz, Sebastian and Hertzberg, Joachim},
  title = {A Spatio-Temporal-Semantic Environment Representation for Autonomous
  Mobile Robots   equipped with   various Sensor Systems},
  booktitle = {2022 IEEE International Conference on Multisensor Fusion and
  Integration for Intelligent Systems (MFI)},
  year = {2022},
  publisher = {IEEE}
}
```

## 📄 License

This project is open-sourced software licensed under the [BSD 3-Clause license](./LICENSE).
