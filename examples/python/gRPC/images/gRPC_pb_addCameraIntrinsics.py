#!/usr/bin/env python3

import os
import sys

from google.protobuf import empty_pb2
from seerep.pb import camera_intrinsics_pb2 as cameraintrinsics
from seerep.pb import camera_intrinsics_query_pb2 as cameraintrinsicsquery
from seerep.pb import camera_intrinsics_service_pb2_grpc as camintrinsics_service
from seerep.pb import meta_operations_pb2_grpc as metaOperations

# importing util functions. Assuming that this file is in the parent dir
# https://github.com/agri-gaia/seerep/blob/6c4da5736d4a893228e97b01a9ada18620b1a83f/examples/python/gRPC/util.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import uuid

from seerep.util.common import get_gRPC_channel

# Default server is localhost !
channel = get_gRPC_channel()

# 1. Get gRPC service objects
stub = camintrinsics_service.CameraIntrinsicsServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

# 2. Get all projects from the server
response = stubMeta.GetProjects(empty_pb2.Empty())

# 3. Check if we have an existing test project, if not, we stop here
projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid + "\n")
    if project.name == "testproject":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()

ciuuid = str(uuid.uuid4())
print("Camera Intrinsics will be saved against the uuid: ", ciuuid)

camin = cameraintrinsics.CameraIntrinsics()

camin.header.stamp.seconds = 4
camin.header.stamp.nanos = 3

camin.header.frame_id = "camintrinsics"

camin.header.uuid_project = projectuuid
camin.header.uuid_msgs = ciuuid

camin.region_of_interest.x_offset = 2
camin.region_of_interest.y_offset = 1
camin.region_of_interest.height = 5
camin.region_of_interest.width = 4
camin.region_of_interest.do_rectify = 4

camin.height = 5
camin.width = 4

camin.distortion_model = "plumb_bob"

camin.distortion.extend([3, 4, 5])

camin.intrinsic_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11])
camin.rectification_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11])
camin.projection_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14])

camin.binning_x = 6
camin.binning_y = 7

camin.maximum_viewing_distance = 5

stub.TransferCameraIntrinsics(camin)

# Fetch the saved CI
ci_query = cameraintrinsicsquery.CameraIntrinsicsQuery()

ci_query.uuid_camera_intrinsics = ciuuid
ci_query.uuid_project = projectuuid

fetched_camintrinsics = stub.GetCameraIntrinsics(ci_query)
print(fetched_camintrinsics)
