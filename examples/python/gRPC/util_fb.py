import sys
import time

import flatbuffers
from fb import (
    Boundingbox,
    BoundingBoxLabeled,
    Empty,
    Header,
    LabelWithInstance,
    Point,
    PointCloud2,
    PointField,
    ProjectCreation,
    ProjectInfo,
    ProjectInfos,
    Timestamp,
)
from fb import meta_operations_grpc_fb as metaOperations


def getOrCreateProject(channel, name, create=True, mapFrameId="map"):
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
        if response.Projects(i).Name().decode("utf-8") == name:
            projectuuid = response.Projects(i).Uuid().decode("utf-8")

    if projectuuid == "":
        if create:
            mapFrameIdBuf = builder.CreateString(mapFrameId)
            nameBuf = builder.CreateString(name)
            ProjectCreation.Start(builder)
            ProjectCreation.AddMapFrameId(builder, mapFrameIdBuf)
            ProjectCreation.AddName(builder, nameBuf)
            projectCreationMsg = ProjectCreation.End(builder)
            builder.Finish(projectCreationMsg)
            buf = builder.Output()

            responseBuf = stubMeta.CreateProject(bytes(buf))
            response = ProjectInfo.ProjectInfo.GetRootAs(responseBuf)

            projectuuid = response.Uuid().decode("utf-8")
        else:
            sys.exit()

    return projectuuid


# Header
def createTimeStamp(builder, seconds, nanoseconds=0):
    '''Create a time stamp in flatbuffers'''
    Timestamp.Start(builder)
    Timestamp.AddSeconds(builder, seconds)
    Timestamp.AddNanos(builder, nanoseconds)
    return Timestamp.End(builder)


def createHeader(builder, timeStamp, frame, projectUuid):
    '''Creates a message header in flatbuffers'''
    frameStr = builder.CreateString(frame)
    projectUuidStr = builder.CreateString(projectUuid)
    Header.Start(builder)
    Header.AddFrameId(builder, frameStr)
    Header.AddStamp(builder, timeStamp)
    Header.AddUuidProject(builder, projectUuidStr)
    return Header.End(builder)


# Point clouds
def createPointField(builder, name, offset, datatype, count):
    '''Creates a point field in flatbuffers to describe a channel in the point cloud'''
    nameStr = builder.CreateString(name)
    PointField.Start(builder)
    PointField.AddName(builder, nameStr)
    PointField.AddOffset(builder, offset)
    PointField.AddDatatype(builder, datatype)
    PointField.AddCount(builder, count)
    return PointField.End(builder)


def createPointFields(builder, channels, datatype, dataTypeOffset, count):
    '''Creates point fields for all specified channels'''
    pointFieldsList = []
    offset = 0
    for channel in channels:
        pointFieldsList.append(createPointField(builder, channel, offset, datatype, count))
        offset += dataTypeOffset
    return pointFieldsList


def createLabelWithInstance(builder, label, instanceUuid):
    '''Creates a label with an associated instance uuid in flatbuffers'''
    labelStr = builder.CreateString(label)
    instanceUuidStr = builder.CreateString(instanceUuid)
    LabelWithInstance.Start(builder)
    LabelWithInstance.AddLabel(builder, labelStr)
    LabelWithInstance.AddInstanceUuid(builder, instanceUuidStr)
    return LabelWithInstance.End(builder)


def createLabelsWithInstance(builder, labels, instanceUuids):
    '''Creates multiple general labels'''
    assert len(labels) == len(instanceUuids)
    labelsGeneral = []
    for label, uuid in zip(labels, instanceUuids):
        labelsGeneral.append(createLabelWithInstance(builder, label, uuid))
    return labelsGeneral


def createPoint(builder, x, y, z):
    '''Creates a 3D point in flatbuffers'''
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    Point.AddZ(builder, z)
    return Point.End(builder)


def createBoundingBox(builder, header, pointMin, pointMax):
    '''Creates a 3D bounding box in flatbuffers'''
    Boundingbox.Start(builder)
    Boundingbox.AddHeader(builder, header)
    Boundingbox.AddPointMin(builder, pointMin)
    Boundingbox.AddPointMax(builder, pointMax)
    return Boundingbox.End(builder)


def createBoundingBoxes(builder, header, minPoints, maxPoints):
    assert len(minPoints) == len(maxPoints)
    boundingBoxes = []
    for pointMin, pointMax in zip(minPoints, maxPoints):
        boundingBoxes.append(createBoundingBox(builder, header, pointMin, pointMax))
    return boundingBoxes


def createBoundingBoxLabeled(builder, instance, boundingBox):
    '''Creates a labeled bounding box in flatbuffers'''
    BoundingBoxLabeled.Start(builder)
    BoundingBoxLabeled.AddLabelWithInstance(builder, instance)
    BoundingBoxLabeled.AddBoundingBox(builder, boundingBox)
    return BoundingBoxLabeled.End(builder)


def createBoundingBoxesLabeled(builder, instances, boundingBoxes):
    '''Creates multiple labeled bounding boxes'''
    assert len(instances) == len(boundingBoxes)
    boundingBoxesLabeled = []
    for instance, boundingBox in zip(instances, boundingBoxes):
        boundingBoxesLabeled.append(createBoundingBoxLabeled(builder, instance, boundingBox))
    return boundingBoxesLabeled


def addToBoundingBoxLabeledVector(builder, boundingBoxLabeledList):
    '''Adds list of boudingBoxLabeled into the labelsBbVector of a flatbuffers pointcloud2'''
    PointCloud2.StartLabelsBbVector(builder, len(boundingBoxLabeledList))
    # Note: reverse because we prepend
    for bb in reversed(boundingBoxLabeledList):
        builder.PrependUOffsetTRelative(bb)
    return builder.EndVector()


def addToGeneralLabelsVector(builder, generalLabelList):
    '''Adds list of generalLabels into the labelsGeneralVector of a flatbuffers pointcloud2'''
    PointCloud2.StartLabelsGeneralVector(builder, len(generalLabelList))
    # Note: reverse because we prepend
    for label in reversed(generalLabelList):
        builder.PrependUOffsetTRelative(label)
    return builder.EndVector()


def addToPointFieldVector(builder, pointFieldList):
    '''Adds a list of pointFields into the fieldsVector of a flatbuffers pointcloud2'''
    PointCloud2.StartFieldsVector(builder, len(pointFieldList))
    # Note: reverse because we prepend
    for pointField in reversed(pointFieldList):
        builder.PrependUOffsetTRelative(pointField)
    return builder.EndVector()
