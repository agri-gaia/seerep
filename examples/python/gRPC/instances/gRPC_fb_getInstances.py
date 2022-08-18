#!/usr/bin/env python3

import sys

import flatbuffers
import grpc
from fb import (
    Boundingbox,
    Empty,
    Header,
    Point,
    ProjectInfos,
    Query,
    QueryInstance,
    TimeInterval,
    Timestamp,
    UuidsPerProject,
)
from fb import instance_service_grpc_fb as instanceService
from fb import meta_operations_grpc_fb as metaOperations

server = "localhost:9090"
channel = grpc.insecure_channel(server)

stub = instanceService.InstanceServiceStub(channel)
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
    print(response.Projects(i).Name().decode("utf-8") + " " + response.Projects(i).Uuid().decode("utf-8"))
    if response.Projects(i).Name().decode("utf-8") == "testproject":
        projectuuid = response.Projects(i).Uuid().decode("utf-8")

if projectuuid == "":
    sys.exit()


Point.Start(builder)
Point.AddX(builder, -100.0)
Point.AddY(builder, -100.0)
Point.AddZ(builder, -100.0)
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


label = builder.CreateString("1")
Query.StartLabelVector(builder, 1)
builder.PrependUOffsetTRelative(label)
labelMsg = builder.EndVector()

Query.Start(builder)
Query.AddBoundingbox(builder, boundingbox)
# Query.AddTimeinterval(builder, timeInterval)
# Query.AddProjectuuid(builder, projectuuidMsg)
# Query.AddLabel(builder, labelMsg)
queryMsg = Query.End(builder)

QueryInstance.Start(builder)
QueryInstance.AddQuery(builder, queryMsg)
queryInstanceMsg = QueryInstance.End(builder)

builder.Finish(queryInstanceMsg)
buf = builder.Output()

responseBuf = stub.GetInstances(bytes(buf))

response = UuidsPerProject.UuidsPerProject.GetRootAs(responseBuf)

print(response.UuidsPerProject(0))
