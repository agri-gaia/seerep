# Package Overview

This page provides an overview of all the ROS-Packages used in SEEREP.

## General SEEREP Structure

The general structure of SEEREP is schematically illustrated in the following
graphic.

SEEREP can be split into two parts, one of which runs on the robot (left box)
and one which runs on a server clusters (right box). The communication is
handled via [gRPC](https://grpc.io/) and [protocol-buffers
(PB)](https://developers.google.com/protocol-buffers/docs/overview) or
[flatbuffers (FB)](https://google.github.io/flatbuffers/).

![SEEREP-Structure](../imgs/SEEREP-Structure.svg){ width=700px }

## Packages

In the following, each package will be described in more detail.

### seerep-hdf5

The `seerep-hdf5` unit provides access to the hdf5 files to store or retrieve
data. The unit is split into three
packages `seerep-hdf5-core`, `seerep-hdf5-pb` and `seerep-hdf5-fb`. This is to
have a server-core which is independent of the message format, so that it's
possible to easily switch between PB and FB or any other message format.

* The main task for the `seerep-core` is to read UUIDs, bounding boxes (BB),
  time/semantic information on provided indices from the hdf5-files. The only
  write operation of the core is to create new hdf5-files. Due to the
  independence of FB and PB, new communication-messages are added to seerep-msgs
  (`seerep-msgs/core`).

* `seerep-hdf5-pb` and `seerep-hdf5-fb` provide methods to read or write
  point clouds, images and transformations from PB or FB  messages.

### seerep-srv

The `seerep-srv` is split into four parts `seerep-server`, `seerep-core`
and `seerep-core-pb`, `seerep-core-fb`.

* The `seerep-server` provides the top level interface for the SEEREP server
  cluster, services which clients can be registered here. The server passes
  request to the corresponding unit in the layer below (see graphic).

* The `seerep-core-pb` / `seerep-core-fb`  writes incoming PB / FB messages to
  the hdf5 files. In case of a query the `seerep-core` is asked for the UUIDs of
  the datasets which match the query parameters.

### seerep-msgs

The `seerep-msgs` package defines all the PB, FB and core messages used in
SEEREP.

### seerep-ros

`seerep-ros` provides three packages which run on the robot itself. The
`seerep_ros_conversions_pb/fb` packages simply convert ROS messages to PB/FB and
vice versa. The second package `seerep_ros_communication` is used to save sensor
information like images and point clouds on the robot, or in case of a good
internet connection to the remote server-cluster. Further, the robot is able to
query the server for information to support his understanding of the environment
and aid its navigation.

* The `seerep_ros_communication\client` is responsible for sending sensor
  information directly to the remote server.

* The `seerep_ros_communication\querier` is used to get information from the
  remote server.

* `seerep_ros_communication\hdf5-dump` is used to save sensor information on a
  hard drive which is located on the robot.

### seerep-com

`seerep-com` is used to define the gRPC services in PB and FB.
