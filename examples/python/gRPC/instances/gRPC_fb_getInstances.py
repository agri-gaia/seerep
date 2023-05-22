#!/usr/bin/env python3

import flatbuffers
from seerep.fb import Datatype, UuidsPerProject
from seerep.fb import instance_service_grpc_fb as instanceService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createBoundingBoxStamped,
    createHeader,
    createLabelWithCategory,
    createPoint,
    createQuery,
    createQueryInstance,
    createTimeInterval,
    createTimeStamp,
    getProject,
)

builder = flatbuffers.Builder(1024)
# Default server is localhost !
channel = get_gRPC_channel()


# 1. Get all projects from the server
projectuuid = getProject(builder, channel, 'testproject')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    exit()

# 3. Get gRPC service object
stub = instanceService.InstanceServiceStub(channel)


# Create all necessary objects for the query
header = createHeader(builder, frame="map")
pointMin = createPoint(builder, 0.0, 0.0, 0.0)
pointMax = createPoint(builder, 100.0, 100.0, 100.0)
boundingboxStamped = createBoundingBoxStamped(builder, header, pointMin, pointMax)

timeMin = createTimeStamp(builder, 1610549273, 0)
timeMax = createTimeStamp(builder, 1938549273, 0)
timeInterval = createTimeInterval(builder, timeMin, timeMax)


projectUuids = [builder.CreateString(projectuuid)]
category = "0"
labels = [[builder.CreateString("testlabel0"), builder.CreateString("testlabelgeneral0")]]
labelCategory = createLabelWithCategory(builder, category, labels)
dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = createQuery(
    builder,
    # boundingBox=boundingboxStamped,
    # timeInterval=timeInterval,
    # labels=labelCategory,
    # projectUuids=projectUuids,
    # instanceUuids=instanceUuids,
    # dataUuids=dataUuids,
    withoutData=True,
)


queryInstanceMsg = createQueryInstance(builder, query, Datatype.Datatype().Image)

builder.Finish(queryInstanceMsg)
buf = builder.Output()

responseBuf = stub.GetInstances(bytes(buf))

response = UuidsPerProject.UuidsPerProject.GetRootAs(responseBuf)

print(response.UuidsPerProject(0).ProjectUuid().decode('utf-8'))
