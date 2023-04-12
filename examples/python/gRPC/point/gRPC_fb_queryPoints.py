#!/usr/bin/env python3

import os
import sys

import flatbuffers
from seerep.fb import PointStamped
from seerep.fb import point_service_grpc_fb as pointService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)

PROJECT_NAME = "simulatedDataWithInstances"
channel = util.get_gRPC_channel()
# 1. Get all projects from the server
projectuuid = util_fb.getProject(builder, channel, 'LabeledImagesInGrid')
# 2. Check if the defined project exist; if not exit
if not projectuuid:
    exit()

# Create all necessary objects for the query
header = util_fb.createHeader(builder, frame="map")
pointMin = util_fb.createPoint(builder, 0.0, 0.0, 0.0)
pointMax = util_fb.createPoint(builder, 100.0, 100.0, 100.0)
boundingboxStamped = util_fb.createBoundingBoxStamped(builder, header, pointMin, pointMax)

timeMin = util_fb.createTimeStamp(builder, 1610549273, 0)
timeMax = util_fb.createTimeStamp(builder, 1938549273, 0)
timeInterval = util_fb.createTimeInterval(builder, timeMin, timeMax)


projectUuids = [builder.CreateString(projectuuid)]
category = "0"

labels = [
    [
        util_fb.createLabelWithConfidence(builder, "testlabel0"),
        util_fb.createLabelWithConfidence(builder, "testlabelgeneral0"),
    ]
]
labelCategory = util_fb.createLabelWithCategory(builder, category, labels)
dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = util_fb.createQuery(
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
