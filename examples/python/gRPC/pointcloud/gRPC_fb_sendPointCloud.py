#!/user/bin/env python3

import time
import uuid

import flatbuffers
import numpy as np

np.set_printoptions(precision=7)
from seerep.fb import PointCloud2, Quaternion, Transform, TransformStamped, Vector3
from seerep.fb import point_cloud_service_grpc_fb as pointCloudService
from seerep.fb import tf_service_grpc_fb
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    addToPointFieldVector,
    createBoundingBoxes,
    createBoundingBoxesLabeled,
    createBoundingBoxLabeledWithCategory,
    createHeader,
    createLabelsWithInstance,
    createLabelWithCategory,
    createPoint,
    createPointFields,
    createTimeStamp,
    getOrCreateProject,
)

NUM_GENERAL_LABELS = 1
NUM_BB_LABELS = 1
NUM_POINT_CLOUDS = 1


def createPointCloud(builder, header, height=960, width=1280):
    '''Creates a flatbuffers point cloud message'''
    pointFields = createPointFields(builder, ['x', 'y', 'z', 'rgba'], 7, 4, 1)
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
        [createPoint(builder, np.random.rand(), np.random.rand(), np.random.rand()) for _ in range(NUM_BB_LABELS)],
        [createPoint(builder, np.random.rand(), np.random.rand(), np.random.rand()) for _ in range(NUM_BB_LABELS)],
    )
    labelWithInstances = createLabelsWithInstance(
        builder,
        ["BoundingBoxLabel" + str(i) for i in range(NUM_BB_LABELS)],
        [i * 10.0 for i in range(NUM_BB_LABELS)],
        [str(uuid.uuid4()) for _ in range(NUM_BB_LABELS)],
    )
    labelsBb = createBoundingBoxesLabeled(builder, labelWithInstances, boundingBoxes)
    labelsBBCat = createBoundingBoxLabeledWithCategory(builder, builder.CreateString("myCategory"), labelsBb)
    # labelsBbVector = addToBoundingBoxLabeledVector(builder, labelsBBCat)

    # Note: rgb field is float, for simplification
    pointsBox = [[]]
    lim = 3
    for a in range(-lim, lim + 1, 1):
        for b in range(-lim, lim + 1, 1):
            for c in range(-lim, lim + 1, 1):
                pointsBox[0].append([a, b, c, 0])
    points = np.array(pointsBox).astype(np.float32)
    print(f"Data: {points}")

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
    # PointCloud2.AddLabelsGeneral(builder, labelsGeneralCat)
    # PointCloud2.AddLabelsBb(builder, labelsBBCat)
    return PointCloud2.End(builder)


def createPointClouds(projectUuid, numOf, theTime):
    '''Creates numOf pointcloud2 messages as a generator function'''

    for i in range(numOf):
        print(f"Send point cloud: {str(i+1)}")
        builder = flatbuffers.Builder(1024)

        timeStamp = createTimeStamp(builder, theTime + i)
        header = createHeader(builder, timeStamp, "scanner", projectUuid)

        pointCloudMsg = createPointCloud(builder, header, 10, 10)
        builder.Finish(pointCloudMsg)
        yield bytes(builder.Output())


def createTF(channel, numOf, projectUuid, theTime):

    for i in range(numOf):

        builderTf = flatbuffers.Builder(1024)

        timeStamp = createTimeStamp(builderTf, theTime + i)
        header = createHeader(builderTf, timeStamp, "map", projectUuid)

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


channel = get_gRPC_channel()
builder = flatbuffers.Builder(1024)
theTime = 1686038855
projectUuid = getOrCreateProject(builder, channel, "testproject")

tfStub = tf_service_grpc_fb.TfServiceStub(channel)
tfStub.TransferTransformStamped(createTF(channel, NUM_POINT_CLOUDS, projectUuid, theTime))

stub = pointCloudService.PointCloudServiceStub(channel)
pc = createPointClouds(projectUuid, NUM_POINT_CLOUDS, theTime)
responseBuf = stub.TransferPointCloud2(pc)
