#!/usr/bin/env python3

import flatbuffers
from seerep.fb import PointStamped
from seerep.fb import point_service_grpc_fb as pointService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createBoundingBoxStamped,
    createHeader,
    createLabelWithCategory,
    createLabelWithConfidence,
    createPoint,
    createQuery,
    createTimeInterval,
    createTimeStamp,
    getProject,
)

builder = flatbuffers.Builder(1024)

PROJECT_NAME = "simulatedDataWithInstances"
channel = get_gRPC_channel()
# 1. Get all projects from the server
projectuuid = getProject(builder, channel, 'LabeledImagesInGrid')
# 2. Check if the defined project exist; if not exit
if not projectuuid:
    exit()

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

labels = [
    [
        createLabelWithConfidence(builder, "testlabel0"),
        createLabelWithConfidence(builder, "testlabelgeneral0"),
    ]
]
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
    withoutData=False,
)

builder.Finish(query)
buf = builder.Output()


stub = pointService.PointServiceStub(channel)
for responseBuf in stub.GetPoint(bytes(buf)):
    response = PointStamped.PointStamped.GetRootAs(responseBuf)

    print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
    for i in range(response.LabelsGeneralLength()):
        for j in range(response.LabelsGeneral(i).LabelsWithInstanceLength()):
            print(f"instance uuid: {response.LabelsGeneral(i).LabelsWithInstance(j).InstanceUuid().decode('utf-8')}")
            print(f"Label: {response.LabelsGeneral(i).LabelsWithInstance(j).Label().Label().decode('utf-8')}")
            print(f"Label confidence: {response.LabelsGeneral(i).LabelsWithInstance(j).Label().Confidence()}")
