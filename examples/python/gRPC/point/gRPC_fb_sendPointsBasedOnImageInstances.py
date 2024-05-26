#!/usr/bin/env python3

import uuid
from typing import Dict, List

import flatbuffers
from grpc import Channel
from seerep.fb import Datatypes, Image, Integer, PointStamped, String, UnionMapEntry
from seerep.fb import image_service_grpc_fb as imageService
from seerep.fb import point_service_grpc_fb as pointService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createHeader,
    createLabelsWithInstanceWithCategory,
    createPoint,
    createQuery,
    createTimeStamp,
    getOrCreateProject,
)


def send_points_raw(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> Dict[str, List[bytearray]]:
    builder = flatbuffers.Builder(1024)

    if target_proj_uuid is None:
        target_proj_uuid = getOrCreateProject(builder, grpc_channel, "testproject")

    stubImage = imageService.ImageServiceStub(grpc_channel)
    stubPoint = pointService.PointServiceStub(grpc_channel)

    queryMsg = createQuery(builder)
    builder.Finish(queryMsg)
    buf = builder.Output()

    bufBytes = []

    img_uuid_point_map: Dict[str, List[bytearray]] = {}

    for responseBuf in stubImage.GetImage(bytes(buf)):
        response = Image.Image.GetRootAs(responseBuf)

        uuid_img = response.Header().UuidMsgs().decode()

        img_uuid_point_map.setdefault(uuid_img, [])

        if not response.LabelsBbIsNone():
            for i in range(response.LabelsBb(0).BoundingBox2dLabeledLength()):
                frameId = response.Header().FrameId().decode("utf-8")
                uuidProject = response.Header().UuidProject().decode("utf-8")
                timestampMsg = createTimeStamp(
                    builder,
                    response.Header().Stamp().Seconds(),
                    response.Header().Stamp().Nanos(),
                )
                header = createHeader(builder, timestampMsg, frameId, uuidProject, str(uuid.uuid4()))

                coordinates = (1, 2, 3)
                point = createPoint(builder, *coordinates)

                label = response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().Label().Label().decode("utf-8")
                confidence = response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().Label().Confidence()
                instance_uuid = (
                    response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().InstanceUuid().decode("utf-8")
                )

                labelWithCat = createLabelsWithInstanceWithCategory(
                    builder, ["myCategory"], [[label]], [[instance_uuid]], [[confidence]]
                )

                PointStamped.StartLabelsGeneralVector(builder, len(labelWithCat))
                for label in reversed(labelWithCat):
                    builder.PrependUOffsetTRelative(label)
                labelWithCat = builder.EndVector()

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
                PointStamped.AddAttribute(builder, attributes)
                PointStamped.AddLabelsGeneral(builder, labelWithCat)
                pointStampedMsg = PointStamped.End(builder)

                builder.Finish(pointStampedMsg)

                buf = builder.Output()

                img_uuid_point_map[uuid_img].append(buf)

                bufBytes.append(bytes(buf))

    stubPoint.TransferPoint(iter(bufBytes))

    return img_uuid_point_map


def send_points(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> Dict[str, List[PointStamped.PointStamped]]:
    img_uuid_point_map: Dict[str, List[PointStamped.PointStamped]] = {}
    for k, v in send_points_raw(target_proj_uuid, grpc_channel).items():
        img_uuid_point_map[k] = [PointStamped.PointStamped.GetRootAs(p) for p in v]
    return img_uuid_point_map


if __name__ == "__main__":
    p_dict = send_points()

    count_points = 0

    for k in p_dict:
        print(f"uuidmsg: {k}")
        for val in p_dict[k]:
            print(f"    uuidlabel: {val.LabelsGeneral(0).LabelsWithInstance(0).Label().Label().decode('utf-8')}")
            print(f"    point_uuidmsg: {val.Header().UuidMsgs().decode('utf-8')}")
            print(f"    point_uuidproject: {val.Header().UuidProject().decode('utf-8')}")
            # check for attribute 0
            if val.Attribute(0).ValueType() == Datatypes.Datatypes().String:
                union_str = String.String()
                union_str.Init(val.Attribute(0).Value().Bytes, val.Attribute(0).Value().Pos)
                print(f"    Attribute 0 Value: {union_str.Data().decode()}\n")
            count_points += 1
    print(f"sent {count_points} points in total")
