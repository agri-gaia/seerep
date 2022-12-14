#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import Boundingbox
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
projectuuid = util_fb.getProject(builder, channel, 'geodeticProject')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    print("Project Not Found")
    exit()

# 3. Get gRPC service object
stub = metaOperations.MetaOperationsStub(channel)

projectInfo = util_fb.createProjectInfo(builder, "name", projectuuid)
builder.Finish(projectInfo)
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
