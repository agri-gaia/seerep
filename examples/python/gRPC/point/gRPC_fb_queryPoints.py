#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import Boundingbox, Header, Point, PointStamped, Query, TimeInterval, Timestamp
from fb import point_service_grpc_fb as pointService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

PROJECT_NAME = "simulatedDataWithInstances"
channel = util.get_gRPC_channel("dev")
projectUuid = util_fb.get_or_create_project(channel, PROJECT_NAME, False)

stub = pointService.PointServiceStub(channel)
builder = flatbuffers.Builder(1024)

Point.Start(builder)
Point.AddX(builder, 0.0)
Point.AddY(builder, 0.0)
Point.AddZ(builder, 0.0)
pointMin = Point.End(builder)

Point.Start(builder)
Point.AddX(builder, 100.0)
Point.AddY(builder, 100.0)
Point.AddZ(builder, 100.0)
pointMax = Point.End(builder)

frameId = builder.CreateString("map")
Header.Start(builder)
Header.AddFrameId(builder, frameId)
header = Header.End(builder)

Boundingbox.Start(builder)
Boundingbox.AddPointMin(builder, pointMin)
Boundingbox.AddPointMax(builder, pointMax)
Boundingbox.AddHeader(builder, header)
boundingbox = Boundingbox.End(builder)

Timestamp.Start(builder)
Timestamp.AddSeconds(builder, 1610549273)
Timestamp.AddNanos(builder, 0)
timeMin = Timestamp.End(builder)

Timestamp.Start(builder)
Timestamp.AddSeconds(builder, 1938549273)
Timestamp.AddNanos(builder, 0)
timeMax = Timestamp.End(builder)

TimeInterval.Start(builder)
TimeInterval.AddTimeMin(builder, timeMin)
TimeInterval.AddTimeMax(builder, timeMax)
timeInterval = TimeInterval.End(builder)

projectuuidString = builder.CreateString(projectUuid)
Query.StartProjectuuidVector(builder, 1)
builder.PrependUOffsetTRelative(projectuuidString)
projectuuidMsg = builder.EndVector()


label = builder.CreateString("testlabel0")
Query.StartLabelVector(builder, 1)
builder.PrependUOffsetTRelative(label)
labelMsg = builder.EndVector()

Query.Start(builder)
Query.AddBoundingbox(builder, boundingbox)
Query.AddTimeinterval(builder, timeInterval)
Query.AddProjectuuid(builder, projectuuidMsg)
Query.AddLabel(builder, labelMsg)
queryMsg = Query.End(builder)

builder.Finish(queryMsg)
buf = builder.Output()

for responseBuf in stub.GetPoint(bytes(buf)):
    response = PointStamped.PointStamped.GetRootAs(responseBuf)

    print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
    for i in range(response.LabelsGeneralLength()):
        print(f"instance uuid: {response.LabelsGeneral(i).InstanceUuid().decode('utf-8')}")
        print(f"Label: {response.LabelsGeneral(i).Label().decode('utf-8')}")
