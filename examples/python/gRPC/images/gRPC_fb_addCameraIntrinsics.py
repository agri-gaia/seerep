#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import camera_intrinsics_service_grpc_fb as ci_service

# importing util functions. Assuming that these files are in the parent dir
# examples/python/gRPC/util.py
# examples/python/gRPC/util_fb.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import uuid

import util
import util_fb

builder = flatbuffers.Builder(1000)
# Default server is localhost !
channel = util.get_gRPC_channel()

# 1. Get all projects from the server
projectuuid = util_fb.getProject(builder, channel, 'testproject')

ciuuid = str(uuid.uuid4())
print("Camera Intrinsics will be saved against the uuid: ", ciuuid)

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    print("project doesn't exist!")
    exit()

# 3. Get gRPC service object
stub = ci_service.CameraIntrinsicsServiceStub(channel)

# Create all necessary objects for the query
ts = util_fb.createTimeStamp(builder, 4, 3)
header = util_fb.createHeader(builder, ts, "map", projectuuid, ciuuid)
roi = util_fb.createRegionOfInterest(builder, 3, 5, 6, 7, True)

matrix = [4, 5, 6]
ci = util_fb.createCameraIntrinsics(builder, header, 3, 4, "plump_bob", matrix, matrix, matrix, matrix, 4, 5, roi)
builder.Finish(ci)

buf = builder.Output()

stub.TransferCameraIntrinsics(bytes(buf))

# Fetch the saved CI
builder = flatbuffers.Builder(1000)

ci_query = util_fb.createCameraIntrinsicsQuery(builder, ciuuid, projectuuid)

builder.Finish(ci_query)
buf = builder.Output()

ret = stub.GetCameraIntrinsics(bytes(buf))

for ci in ret:
    print(ci)
