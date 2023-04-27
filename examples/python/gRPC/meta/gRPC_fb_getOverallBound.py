#!/usr/bin/env python3

import os
import sys

import flatbuffers
from seerep.fb import Boundingbox, Datatype, TimeInterval
from seerep.fb import meta_operations_grpc_fb as metaOperations

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
projectuuid = util_fb.getProject(builder, channel, 'testproject')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    print("Project Not Found")
    exit()

# 3. Get gRPC service object
stub = metaOperations.MetaOperationsStub(channel)

UuidDatatypePair = util_fb.createUuidDatatypePair(builder, projectuuid, Datatype.Datatype().Image)

builder.Finish(UuidDatatypePair)
buf = builder.Output()

responseBuf = stub.GetOverallBoundingBox(bytes(buf))
response = Boundingbox.Boundingbox.GetRootAs(responseBuf)

print(
    "Center Point (X, Y, Z): "
    + str(response.CenterPoint().X())
    + " , "
    + str(response.CenterPoint().Y())
    + " , "
    + str(response.CenterPoint().Z())
)

print(
    "Spatial Things Point (X, Y, Z): "
    + str(response.SpatialExtent().X())
    + " , "
    + str(response.SpatialExtent().Y())
    + " , "
    + str(response.SpatialExtent().Z())
)

responseBuf = stub.GetOverallTimeInterval(bytes(buf))
response = TimeInterval.TimeInterval.GetRootAs(responseBuf)

print(
    " Minimum Time "
    + str(response.TimeMin().Seconds())
    + "s and "
    + str(response.TimeMin().Nanos())
    + "ms\n"
    + " Maximum Time "
    + str(response.TimeMax().Seconds())
    + "s and "
    + str(response.TimeMax().Nanos())
    + "ms"
)
