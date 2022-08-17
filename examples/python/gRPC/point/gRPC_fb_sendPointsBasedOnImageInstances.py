#!/usr/bin/env python3

import os
import sys

import flatbuffers
import grpc
from fb import (
    Boundingbox,
    BoundingBox2DLabeled,
    Datatypes,
    Empty,
    Header,
    Image,
    Integer,
    LabelWithInstance,
    Point,
    PointStamped,
    ProjectInfos,
    Query,
    String,
    TimeInterval,
    Timestamp,
    UnionMapEntry,
)
from fb import image_service_grpc_fb as imageService
from fb import point_service_grpc_fb as pointService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

channel = util.get_gRPC_channel("local")

projectuuid = util_fb.get_or_create_project(channel, "LabeledImagesInGrid", True)

stubImage = imageService.ImageServiceStub(channel)
stubPoint = pointService.PointServiceStub(channel)


builder = flatbuffers.Builder(1024)

Query.Start(builder)
queryMsg = Query.End(builder)

builder.Finish(queryMsg)
buf = builder.Output()

bufBytes = []

for responseBuf in stubImage.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)
    print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
    if not response.LabelsBbIsNone():
        for i in range(response.LabelsBbLength()):
            print(f"uuidlabel: {response.LabelsBb(i).LabelWithInstance().InstanceUuid().decode('utf-8')}")
            frameId = builder.CreateString(response.Header().FrameId().decode('utf-8'))
            uuidProject = builder.CreateString(response.Header().UuidProject().decode('utf-8'))

            Timestamp.Start(builder)
            Timestamp.AddSeconds(builder, response.Header().Stamp().Seconds())
            Timestamp.AddNanos(builder, response.Header().Stamp().Nanos())
            timestampMsg = Timestamp.End(builder)

            Header.Start(builder)
            Header.AddFrameId(builder, frameId)
            Header.AddStamp(builder, timestampMsg)
            Header.AddUuidProject(builder, uuidProject)
            header = Header.End(builder)

            Point.Start(builder)
            Point.AddX(builder, 1)
            Point.AddY(builder, 2)
            Point.AddZ(builder, 3)
            point = Point.End(builder)

            label = builder.CreateString(response.LabelsBb(i).LabelWithInstance().Label().decode('utf-8'))
            instanceUuid = builder.CreateString(response.LabelsBb(i).LabelWithInstance().InstanceUuid().decode('utf-8'))
            LabelWithInstance.Start(builder)
            LabelWithInstance.AddLabel(builder, label)
            LabelWithInstance.AddInstanceUuid(builder, instanceUuid)
            labelWithInstance = LabelWithInstance.End(builder)

            PointStamped.StartLabelsGeneralVector(builder, 1)
            builder.PrependUOffsetTRelative(labelWithInstance)
            labelWithInstanceMsg = builder.EndVector()

            unionMapEntryKey1 = builder.CreateString("exampleKey1")
            value1String = builder.CreateString("exampleValue1")
            String.Start(builder)
            String.AddData(builder, value1String)
            unionMapEntryValue1 = String.End(builder)
            unionMapEntryKey2 = builder.CreateString("exampleKey2")
            Integer.Start(builder)
            Integer.AddData(builder, 42)
            unionMapEntryValue2 = Integer.End(builder)

            UnionMapEntry.Start(builder)
            UnionMapEntry.AddKey(builder, unionMapEntryKey1)
            UnionMapEntry.UnionMapEntryAddValueType(builder, Datatypes.Datatypes.String)
            UnionMapEntry.AddValue(builder, unionMapEntryValue1)
            unionMapEntry1 = UnionMapEntry.End(builder)
            UnionMapEntry.Start(builder)
            UnionMapEntry.AddKey(builder, unionMapEntryKey2)
            UnionMapEntry.UnionMapEntryAddValueType(builder, Datatypes.Datatypes.Integer)
            UnionMapEntry.AddValue(builder, unionMapEntryValue2)
            unionMapEntry2 = UnionMapEntry.End(builder)

            PointStamped.StartAttributeVector(builder, 2)
            builder.PrependUOffsetTRelative(unionMapEntry1)
            builder.PrependUOffsetTRelative(unionMapEntry2)
            attributes = builder.EndVector()

            PointStamped.Start(builder)
            PointStamped.AddHeader(builder, header)
            PointStamped.AddPoint(builder, point)
            # PointStamped.AddAttribute(builder, attributes)
            PointStamped.AddLabelsGeneral(builder, labelWithInstanceMsg)
            pointStampedMsg = PointStamped.End(builder)

            builder.Finish(pointStampedMsg)
            buf = builder.Output()
            bufBytes.append(bytes(buf))

print("sending points")
stubPoint.TransferPoint(iter(bufBytes))
