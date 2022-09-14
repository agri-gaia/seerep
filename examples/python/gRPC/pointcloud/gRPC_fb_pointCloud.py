#!/user/bin/env python3

import os
import sys
import time

import flatbuffers
import numpy as np
from fb import PointCloud2
from fb import point_cloud_service_grpc_fb as pointCloudService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)

import util
from util_fb import (
    addToBoundingBoxLabeledVector,
    addToGeneralLabelsVector,
    addToPointFieldVector,
    createBoundingBoxes,
    createBoundingBoxesLabeled,
    createHeader,
    createLabelsWithInstance,
    createPoint,
    createPointFields,
    createTimeStamp,
    getOrCreateProject,
)

NUM_GENERAL_LABELS = 10
NUM_BB_LABELS = 10
NUM_POINT_CLOUDS = 10


def createPointCloud(builder, header, height=960, width=1280):
    '''Creates a flatbuffers point cloud message'''
    pointFields = createPointFields(builder, "xyz", 7, 4, 1)
    pointFieldsVector = addToPointFieldVector(builder, pointFields)

    # create general labels
    labelsGeneral = createLabelsWithInstance(
        builder,
        ["GeneralLabel" + str(i) for i in range(NUM_GENERAL_LABELS)],
        ["InstanceUuid" + str(i) for i in range(NUM_GENERAL_LABELS)],
    )
    labelsGeneralVector = addToGeneralLabelsVector(builder, labelsGeneral)

    # create bounding box labels
    boundingBoxes = createBoundingBoxes(
        builder,
        header,
        [createPoint(builder, np.random.rand(), np.random.rand(), np.random.rand()) for i in range(NUM_BB_LABELS)],
        [createPoint(builder, np.random.rand(), np.random.rand(), np.random.rand()) for i in range(NUM_BB_LABELS)],
    )
    labelWithInstances = createLabelsWithInstance(
        builder,
        ["BoundingBoxLabel" + str(i) for i in range(NUM_BB_LABELS)],
        ["InstanceUuid" + str(i) for i in range(NUM_BB_LABELS)],
    )
    labelsBb = createBoundingBoxesLabeled(builder, labelWithInstances, boundingBoxes)
    labelsBbVector = addToBoundingBoxLabeledVector(builder, labelsBb)

    # create ordered point cloud with dim (height, width, 3)
    points = np.random.randn(height, width, 3).astype(np.float32)
    pointsVector = builder.CreateByteVector(points.tobytes())

    # add all data into the flatbuffers point cloud message
    PointCloud2.Start(builder)
    PointCloud2.AddHeader(builder, header)
    PointCloud2.AddHeight(builder, points.shape[0])
    PointCloud2.AddWidth(builder, points.shape[1])
    PointCloud2.AddIsBigendian(builder, False)
    PointCloud2.AddPointStep(builder, 12)
    PointCloud2.AddRowStep(builder, points.shape[1] * 12)
    PointCloud2.AddFields(builder, pointFieldsVector)
    PointCloud2.AddData(builder, pointsVector)
    PointCloud2.AddLabelsGeneral(builder, labelsGeneralVector)
    PointCloud2.AddLabelsBb(builder, labelsBbVector)
    return PointCloud2.End(builder)


def createPointClouds(projectUuid, numOf):
    '''Creates numOf pointcloud2 messages as a generator function'''
    theTime = int(time.time())
    for i in range(numOf):
        print(f"Send point cloud: {str(i+1)}")
        builder = flatbuffers.Builder(1024)

        timeStamp = createTimeStamp(builder, theTime + i)
        header = createHeader(builder, timeStamp, "camera", projectUuid)

        pointCloudMsg = createPointCloud(builder, header)
        builder.Finish(pointCloudMsg)
        yield bytes(builder.Output())


channel = util.get_gRPC_channel()
stub = pointCloudService.PointCloudServiceStub(channel)

projectUuid = getOrCreateProject(channel, "testproject", True)

responseBuf = stub.TransferPointCloud2(createPointClouds(projectUuid, NUM_POINT_CLOUDS))
