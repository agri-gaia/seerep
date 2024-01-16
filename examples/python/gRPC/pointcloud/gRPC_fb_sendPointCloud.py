#!/user/bin/env python3
import struct
import uuid
from typing import List

import flatbuffers
import numpy as np
from grpc import Channel
from quaternion import quaternion
from seerep.fb import PointCloud2, Quaternion, Transform, TransformStamped, Vector3
from seerep.fb import point_cloud_service_grpc_fb as pointCloudService
from seerep.fb import tf_service_grpc_fb
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    addToBoundingBoxLabeledVector,
    addToPointFieldVector,
    createBoundingBoxes,
    createBoundingBoxesLabeled,
    createBoundingBoxLabeledWithCategory,
    createHeader,
    createLabelsWithInstance,
    createLabelWithCategory,
    createPoint,
    createPointFields,
    createQuaternion,
    createTimeStamp,
    getOrCreateProject,
)

NUM_GENERAL_LABELS = 10
NUM_BB_LABELS = 1
NUM_POINT_CLOUDS = 10


def createPointCloud(builder, header, height=960, width=1280):
    """Creates a flatbuffers point cloud message"""
    pointFields = createPointFields(builder, ["x", "y", "z", "rgba"], 7, 4, 1)
    pointFieldsVector = addToPointFieldVector(builder, pointFields)

    # create general labels
    labelsGeneral = createLabelsWithInstance(
        builder,
        ["GeneralLabel" + str(i) for i in range(NUM_GENERAL_LABELS)],
        [i * 10.0 for i in range(NUM_GENERAL_LABELS)],
        [str(uuid.uuid4()) for _ in range(NUM_GENERAL_LABELS)],
    )
    labelsGeneralCat = createLabelWithCategory(builder, ["myCategory"], [labelsGeneral])

    # create bounding box labels
    boundingBoxes = createBoundingBoxes(
        builder,
        [
            createPoint(builder, np.random.rand(), np.random.rand(), np.random.rand())
            for _ in range(NUM_BB_LABELS)
        ],
        [
            createPoint(builder, np.random.rand(), np.random.rand(), np.random.rand())
            for _ in range(NUM_BB_LABELS)
        ],
        [
            createQuaternion(builder, quaternion(1.0, 0.0, 0.0, 0.0))
            for _ in range(NUM_BB_LABELS)
        ],
    )
    labelWithInstances = createLabelsWithInstance(
        builder,
        ["BoundingBoxLabel" + str(i) for i in range(NUM_BB_LABELS)],
        [i * 10.0 for i in range(NUM_BB_LABELS)],
        [str(uuid.uuid4()) for _ in range(NUM_BB_LABELS)],
    )
    labelsBb = createBoundingBoxesLabeled(builder, labelWithInstances, boundingBoxes)
    labelsBBCat = createBoundingBoxLabeledWithCategory(
        builder, builder.CreateString("myCategory"), labelsBb
    )
    labelsBbVector = addToBoundingBoxLabeledVector(builder, [labelsBBCat])

    # Note: rgb field is float, for simplification
    pointsBox = [[]]
    lim = 3
    for a in range(-lim, lim + 1, 1):
        for b in range(-lim, lim + 1, 1):
            for c in range(-lim, lim + 1, 1):
                pointsBox[0].append([a, b, c, 0])
    points = np.array(pointsBox).astype(np.float32)

    pointsVector = builder.CreateByteVector(points.tobytes())

    # add all data into the flatbuffers point cloud message
    PointCloud2.Start(builder)
    PointCloud2.AddHeader(builder, header)
    PointCloud2.AddHeight(builder, points.shape[0])
    PointCloud2.AddWidth(builder, points.shape[1])
    PointCloud2.AddIsBigendian(builder, True)
    PointCloud2.AddPointStep(builder, 16)
    PointCloud2.AddRowStep(builder, points.shape[1] * 16)
    PointCloud2.AddFields(builder, pointFieldsVector)
    PointCloud2.AddData(builder, pointsVector)
    PointCloud2.AddLabelsGeneral(builder, labelsGeneralCat)
    PointCloud2.AddLabelsBb(builder, labelsBbVector)
    return PointCloud2.End(builder)


def createPointClouds(projectUuid, numOf, theTime):
    """Creates numOf pointcloud2 messages as a generator function"""

    for i in range(numOf):
        builder = flatbuffers.Builder(1024)

        timeStamp = createTimeStamp(builder, theTime + i)
        header = createHeader(
            builder, timeStamp, "scanner", projectUuid, str(uuid.uuid4())
        )

        pointCloudMsg = createPointCloud(builder, header, 10, 10)
        builder.Finish(pointCloudMsg)
        yield bytes(builder.Output())


def createTF(numOf, projectUuid, theTime):

    for i in range(numOf):

        builderTf = flatbuffers.Builder(1024)

        timeStamp = createTimeStamp(builderTf, theTime + i)
        header = createHeader(
            builderTf, timeStamp, "map", projectUuid, str(uuid.uuid4())
        )

        Vector3.Start(builderTf)
        Vector3.AddX(builderTf, 10 * i)
        Vector3.AddY(builderTf, 10 * i)
        Vector3.AddZ(builderTf, 10 * i)
        trans = Vector3.End(builderTf)

        # [ x: 45, y: 45, z: 45 ]
        Quaternion.Start(builderTf)
        Quaternion.AddX(builderTf, 0.4619398)
        Quaternion.AddY(builderTf, 0.1913417)
        Quaternion.AddZ(builderTf, 0.4619398)
        Quaternion.AddW(builderTf, 0.7325378)
        rot = Quaternion.End(builderTf)

        Transform.Start(builderTf)
        Transform.AddTranslation(builderTf, trans)
        Transform.AddRotation(builderTf, rot)
        tf = Transform.End(builderTf)

        childFrame = builderTf.CreateString("scanner")

        TransformStamped.Start(builderTf)
        TransformStamped.AddHeader(builderTf, header)
        TransformStamped.AddChildFrameId(builderTf, childFrame)
        TransformStamped.AddTransform(builderTf, tf)
        tfStamped = TransformStamped.End(builderTf)
        builderTf.Finish(tfStamped)
        yield bytes(builderTf.Output())


def send_pointcloud(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[PointCloud2.PointCloud2]:
    builder = flatbuffers.Builder(1024)
    theTime = 1686038855

    if target_proj_uuid == None:
        target_proj_uuid = getOrCreateProject(builder, grpc_channel, "testproject")

    tfStub = tf_service_grpc_fb.TfServiceStub(grpc_channel)
    tfStub.TransferTransformStamped(
        createTF(NUM_POINT_CLOUDS, target_proj_uuid, theTime)
    )

    stub = pointCloudService.PointCloudServiceStub(grpc_channel)
    pc = createPointClouds(target_proj_uuid, NUM_POINT_CLOUDS, theTime)

    pc_list: List = [p for p in pc]

    # one could directly pass the generator, e.g.
    # responseBuf = stub.TransferPointCloud2(pc)
    # but this would consume the generator and then the pc list cannot be returned, therefore use a iterator
    responseBuf = stub.TransferPointCloud2(iter(pc_list))

    pc_list = [PointCloud2.PointCloud2.GetRootAs(p) for p in pc_list]

    return pc_list


if __name__ == "__main__":
    np.set_printoptions(precision=7)
    pc_list = send_pointcloud()
    for idx, pc in enumerate(pc_list):
        print("Num of pc: ", idx + 1)
        raw_data = pc.DataAsNumpy()
        data_flattened = [
            struct.unpack("f", pc.DataAsNumpy()[i : i + pc.FieldsLength()])
            for i in range(0, raw_data.shape[0], pc.FieldsLength())
        ]
        # reshape the array according to its dimensions
        data = np.array(data_flattened).reshape(
            pc.Height(), pc.Width(), pc.FieldsLength()
        )

        print(f"Data: {data}")
