#!/usr/bin/env python3

import os
import sys

import flatbuffers
from seerep.fb import (
    Datatypes,
    Header,
    Image,
    Integer,
    LabelWithInstance,
    Point,
    PointStamped,
    String,
    Timestamp,
    UnionMapEntry,
)
from seerep.fb import image_service_grpc_fb as imageService
from seerep.fb import point_service_grpc_fb as pointService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

channel = util.get_gRPC_channel()

builder = flatbuffers.Builder(1024)
projectuuid = util_fb.getOrCreateProject(builder, channel, "LabeledImagesInGrid")

stubImage = imageService.ImageServiceStub(channel)
stubPoint = pointService.PointServiceStub(channel)

queryMsg = util_fb.createQuery(builder)
builder.Finish(queryMsg)
buf = builder.Output()

bufBytes = []

for responseBuf in stubImage.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)
    print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
    if not response.LabelsBbIsNone():
        for i in range(response.LabelsBb(0).BoundingBox2dLabeledLength()):
            print(
                f"uuidlabel: {response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().InstanceUuid().decode('utf-8')}"
            )

            frameId = response.Header().FrameId().decode('utf-8')
            uuidProject = response.Header().UuidProject().decode('utf-8')
            timestampMsg = util_fb.createTimeStamp(
                builder, response.Header().Stamp().Seconds(), response.Header().Stamp().Nanos()
            )
            header = util_fb.createHeader(builder, timestampMsg, frameId, uuidProject)

            point = util_fb.createPoint(builder, 1, 2, 3)

            labelWithInstanceMsg = util_fb.createLabelWithInstance(
                builder,
                response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().Label().Label().decode('utf-8'),
                response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().Label().Confidence(),
                response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().InstanceUuid().decode('utf-8'),
            )

            labelWithCat = util_fb.createLabelWithCategory(builder, ["myCategory"], [[labelWithInstanceMsg]])

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
            PointStamped.AddLabelsGeneral(builder, labelWithCat)
            pointStampedMsg = PointStamped.End(builder)

            builder.Finish(pointStampedMsg)
            buf = builder.Output()
            bufBytes.append(bytes(buf))

print("sending points")
stubPoint.TransferPoint(iter(bufBytes))
