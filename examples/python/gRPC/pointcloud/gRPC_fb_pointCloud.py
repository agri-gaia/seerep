#!/user/bin/env python3

import os
import sys
import time

import flatbuffers
import numpy as np
from fb import Header, PointCloud2, PointField, Timestamp
from fb import point_cloud_service_grpc_fb as pointCloudService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb


def createTimeStamp(builder, time):
    '''Create a flatbuffers time stamp'''
    Timestamp.Start(builder)
    Timestamp.AddSeconds(builder, time)
    Timestamp.AddNanos(builder, 0)
    return Timestamp.End(builder)


# ToDo add doc for input parameter
def createPointField(builder, name, offset, datatype, count):
    '''Creates point field to describe a channel in the point cloud'''
    nameStr = builder.CreateString(name)
    PointField.Start(builder)
    PointField.AddName(builder, nameStr)
    PointField.AddOffset(builder, offset)
    PointField.AddDatatype(builder, datatype)
    PointField.AddCount(builder, count)
    return PointField.End(builder)


def createPointFields(builder, fields="xyz", datatype=7, count=1):
    '''Creates fields for all channels of the point cloud'''
    fieldsList = []
    offset = 0
    for field in fields:
        fieldsList.append(createPointField(builder, field, offset, datatype, count))
        offset += 4
    return fieldsList


def listToPointFieldVector(builder, pointFieldList):
    '''Creates flatbuffers vector from a list of point fields'''
    PointCloud2.StartFieldsVector(builder, len(pointFieldList))
    for pointField in pointFieldList:
        builder.PrependUOffsetTRelative(pointField)
    return builder.EndVector()


def createHeader(builder, time, frame, projectUuid):
    '''Creates a flatbuffers header'''
    frameIdStr = builder.CreateString(frame)
    projectUuidStr = builder.CreateString(projectUuid)
    timeStamp = createTimeStamp(builder, time)
    Header.Start(builder)
    Header.AddFrameId(builder, frameIdStr)
    Header.AddStamp(builder, timeStamp)
    Header.AddUuidProject(builder, projectUuidStr)
    return Header.End(builder)


def createPointCloud(builder, header):
    '''Creates the point cloud message'''
    pointFields = createPointFields(builder)
    pointFieldVector = listToPointFieldVector(builder, pointFields)

    # generate the actual point cloud data
    points = np.random.randn(10, 3).astype(np.float32)
    pointsVector = builder.CreateByteVector(points.tobytes())

    PointCloud2.Start(builder)
    PointCloud2.AddHeader(builder, header)
    PointCloud2.AddHeight(builder, points.shape[0])
    PointCloud2.AddWidth(builder, points.shape[1])
    PointCloud2.AddIsBigendian(builder, False)
    PointCloud2.AddPointStep(builder, 8)
    PointCloud2.AddRowStep(builder, points.shape[0] * points.shape[1])
    PointCloud2.AddFields(builder, pointFieldVector)
    PointCloud2.AddData(builder, pointsVector)
    return PointCloud2.End(builder)


def createPointClouds(projectUuid, numOf=10):
    '''Creates 'numOf' point cloud messages'''
    theTime = int(time.time())
    for i in range(numOf):
        print(f"Send point cloud: {str(i)}")
        builder = flatbuffers.Builder(1024)

        header = createHeader(builder, theTime + i, "camera", projectUuid)
        pointCloudMsg = createPointCloud(builder, header)
        builder.Finish(pointCloudMsg)
        yield bytes(builder.Output())


channel = util.get_gRPC_channel()
stub = pointCloudService.PointCloudServiceStub(channel)

projectUuid = util_fb.get_or_create_project(channel, "testproject", True)

responseBuf = stub.TransferPointCloud2(createPointClouds(projectUuid))
