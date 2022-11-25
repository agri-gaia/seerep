#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import Datatype, UuidsPerProject
from fb import instance_service_grpc_fb as instanceService

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
    exit()

# 3. Get gRPC service object
stub = instanceService.InstanceServiceStub(channel)


# Create all necessary objects for the query
header = util_fb.createHeader(builder, frame="map")
pointMin = util_fb.createPoint(builder, 0.0, 0.0, 0.0)
pointMax = util_fb.createPoint(builder, 100.0, 100.0, 100.0)
boundingboxStamped = util_fb.createBoundingBoxStamped(builder, header, pointMin, pointMax)

timeMin = util_fb.createTimeStamp(builder, 1610549273, 0)
timeMax = util_fb.createTimeStamp(builder, 1938549273, 0)
timeInterval = util_fb.createTimeInterval(builder, timeMin, timeMax)


projectUuids = [builder.CreateString(projectuuid)]
labels = [builder.CreateString("testlabel0")]
dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = util_fb.createQuery(
    builder,
    # boundingBox=boundingboxStamped,
    # timeInterval=timeInterval,
    # labels=labels,
    # projectUuids=projectUuids,
    # instanceUuids=instanceUuids,
    # dataUuids=dataUuids,
    withoutData=True,
)


queryInstanceMsg = util_fb.createQueryInstance(builder, query, Datatype.Datatype().Image)

builder.Finish(queryInstanceMsg)
buf = builder.Output()

responseBuf = stub.GetInstances(bytes(buf))

response = UuidsPerProject.UuidsPerProject.GetRootAs(responseBuf)

print(response.UuidsPerProject(0).ProjectUuid().decode('utf-8'))
