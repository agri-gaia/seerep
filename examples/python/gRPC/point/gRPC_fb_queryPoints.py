#!/usr/bin/env python3
import sys
from typing import List

import flatbuffers
from seerep.fb import Datatypes, PointStamped, String
from seerep.fb import point_service_grpc_fb as pointService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createLabelsWithCategoryVector,
    createPoint2d,
    createPolygon2D,
    createQuery,
    createTimeInterval,
    createTimeStamp,
    getProject,
)


def get_points_raw(target_proj_uuid: str = None, grpc_channel=get_gRPC_channel()) -> List[bytearray]:
    builder = flatbuffers.Builder(1024)

    # 1. Get all projects from the server
    if target_proj_uuid is None:
        target_proj_uuid = getProject(builder, grpc_channel, "testproject")
        # 2. Check if the defined project exist; if not exit
        if target_proj_uuid is None:
            print("Project does not exist")
            sys.exit()

    # Create all necessary objects for the query
    scale = 200
    vertices = [
        createPoint2d(builder, x * scale, y * scale) for x, y in [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)]
    ]
    polygon2d = createPolygon2D(builder, 36, 0, vertices)

    # ruff: noqa: F841
    timeMin = createTimeStamp(builder, 1610549273, 0)
    timeMax = createTimeStamp(builder, 1938549273, 0)
    timeInterval = createTimeInterval(builder, timeMin, timeMax)

    projectUuids = [builder.CreateString(target_proj_uuid)]
    category = ["0"]

    labels = [["testlabel0", "testlabelgeneral0"]]
    labelCategory = createLabelsWithCategoryVector(builder, category, labels)
    dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
    instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

    # 4. Create a query with parameters
    # all parameters are optional
    # with all parameters set (especially with the data and instance uuids set) the result of the query will be empty.
    # Set the query parameters to adequate values or remove them from the query creation
    query = createQuery(
        builder,
        # timeInterval=timeInterval,
        # labels=labelCategory,
        projectUuids=projectUuids,
        # instanceUuids=instanceUuids,
        # dataUuids=dataUuids,
        polygon2d=polygon2d,
        # withoutData=True,
        fullyEncapsulated=False,
        inMapFrame=True,
    )

    builder.Finish(query)
    buf = builder.Output()

    stub = pointService.PointServiceStub(grpc_channel)
    pointsBuf: List[bytearray] = stub.GetPoint(bytes(buf))

    return pointsBuf


def get_points(target_proj_uuid: str = None, grpc_channel=get_gRPC_channel()) -> List[PointStamped.PointStamped]:
    return [
        PointStamped.PointStamped.GetRootAs(responseBuf)
        for responseBuf in get_points_raw(target_proj_uuid, grpc_channel)
    ]


if __name__ == "__main__":
    p_list = get_points()
    for point in p_list:
        print(f"uuidmsg: {point.Header().UuidMsgs().decode('utf-8')}")
        for i in range(point.LabelsGeneralLength()):
            for j in range(point.LabelsGeneral(i).LabelsWithInstanceLength()):
                print(
                    f"    instance uuid: {point.LabelsGeneral(i).LabelsWithInstance(j).InstanceUuid().decode('utf-8')}"
                )
                print(f"    Label: {point.LabelsGeneral(i).LabelsWithInstance(j).Label().Label().decode('utf-8')}")
                print(f"    Label confidence: {point.LabelsGeneral(i).LabelsWithInstance(j).Label().Confidence()}")
                print(f"   AttributeLen: {point.AttributeLength()}")
        # check for attribute 0
        if point.Attribute(0).ValueType() == Datatypes.Datatypes().String:
            union_str = String.String()
            union_str.Init(point.Attribute(0).Value().Bytes, point.Attribute(0).Value().Pos)
        print(f"Attribute 0 Key: {point.Attribute(0).Key().decode()}")
        print(f"Attribute 0 Value: {union_str.Data().decode()}\n")
    print(f"count of queried pictures: {len(p_list)}")
