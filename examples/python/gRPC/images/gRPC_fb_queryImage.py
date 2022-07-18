#!/usr/bin/env python3

import os
import sys

import flatbuffers
import grpc
from fb import (
    Boundingbox,
    Empty,
    Header,
    Image,
    Point,
    ProjectInfos,
    Query,
    TimeInterval,
    Timestamp,
)
from fb import image_service_grpc_fb as imageService
from fb import meta_operations_grpc_fb as metaOperations

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

builder = flatbuffers.Builder(1024)
Empty.Start(builder)
emptyMsg = Empty.End(builder)
builder.Finish(emptyMsg)
buf = builder.Output()

responseBuf = stubMeta.GetProjects(bytes(buf))
response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

projectuuid = ""
for i in range(response.ProjectsLength()):
    print(response.Projects(i).Name().decode("utf-8") + " " + response.Projects(i).Uuid().decode("utf-8") + "\n")
    if response.Projects(i).Name().decode("utf-8") == "testproject":
        projectuuid = response.Projects(i).Uuid().decode("utf-8")

if projectuuid == "":
    sys.exit()

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

projectuuidString = builder.CreateString(projectuuid)
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

for responseBuf in stub.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)

    print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
    print("first label: " + response.LabelsBb(0).LabelWithInstance().Label().decode("utf-8"))
    print(
        "first bounding box (Xmin,Ymin,Xmax,Ymax): "
        + str(response.LabelsBb(0).BoundingBox().PointMin().X())
        + " "
        + str(response.LabelsBb(0).BoundingBox().PointMin().Y())
        + " "
        + str(response.LabelsBb(0).BoundingBox().PointMax().X())
        + " "
        + str(response.LabelsBb(0).BoundingBox().PointMax().Y())
        + "\n"
    )
