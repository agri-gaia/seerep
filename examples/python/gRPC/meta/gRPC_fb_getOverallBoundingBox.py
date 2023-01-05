#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import Boundingbox, Datatype
from fb import meta_operations_grpc_fb as metaOperations

# importing util functions. Assuming that these files are in the parent dir
# examples/python/gRPC/util.py
# examples/python/gRPC/util_fb.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)
# Default server is localhost !
channel = util.get_gRPC_channel()


# 1. Get all projects from the server
projectuuid = util_fb.getProject(builder, channel, 'LabeledImagesInGrid')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    print("Project Not Found")
    exit()

# 3. Get gRPC service object
stub = metaOperations.MetaOperationsStub(channel)

# 4. Create an instance of fb.Datatype
requested_datatype = Datatype.Datatype()

UuidDatatypePair = util_fb.createUuidDatatypePair(
    builder, projectuuid, [requested_datatype.Image, requested_datatype.PointCloud]
)
builder.Finish(UuidDatatypePair)
buf = builder.Output()

responseBuf = stub.GetOverallBoundingBox(bytes(buf))
response = Boundingbox.Boundingbox.GetRootAs(responseBuf)

print(
    "Min Point (X, Y, Z): "
    + str(response.PointMin().X())
    + " , "
    + str(response.PointMin().Y())
    + " , "
    + str(response.PointMin().Z())
)
print(
    "Max Point (X, Y, Z): "
    + str(response.PointMax().X())
    + " , "
    + str(response.PointMax().Y())
    + " , "
    + str(response.PointMax().Z())
)
