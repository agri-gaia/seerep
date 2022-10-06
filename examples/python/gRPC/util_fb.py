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
    Query,
    TimeInterval,
    Timestamp,
)
from fb import meta_operations_grpc_fb as metaOperations


def getProject(builder, channel, name):
    '''Retrieve a project by name'''
    stubMeta = metaOperations.MetaOperationsStub(channel)

    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stubMeta.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    for i in range(response.ProjectsLength()):
        if response.Projects(i).Name().decode("utf-8") == name:
            return response.Projects(i).Uuid().decode("utf-8")
    return None


def createProject(channel, builder, name, frameId):
    '''Create a project from the parameters'''
    stubMeta = metaOperations.MetaOperationsStub(channel)

    frameIdBuf = builder.CreateString(frameId)
    nameBuf = builder.CreateString(name)

    ProjectCreation.Start(builder)
    ProjectCreation.AddMapFrameId(builder, frameIdBuf)
    ProjectCreation.AddName(builder, nameBuf)
    projectCreationMsg = ProjectCreation.End(builder)
    builder.Finish(projectCreationMsg)

    buf = builder.Output()

    responseBuf = stubMeta.CreateProject(bytes(buf))
    response = ProjectInfo.ProjectInfo.GetRootAs(responseBuf)

    return response.Uuid().decode("utf-8")


def getOrCreateProject(builder, channel, name, create=True, mapFrameId="map"):
    '''Get the project,, or if not present, create one'''
    projectUuid = getProject(builder, channel, name)

    if projectUuid is None:
        if create:
            projectUuid = createProject(channel, builder, name, mapFrameId)
        else:
            sys.exit()

    return projectUuid


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


def createQuery(builder, projectUuids, timeInterval, generalLabels, withoutData=False):
    # add project uuids
    Query.StartProjectuuidVector(builder, len(projectUuids))
    for projectUuid in reversed(projectUuids):
        builder.PrependUOffsetTRelative(projectUuid)
    projectUuidsOffset = builder.EndVector()

    Query.StartLabelVector(builder, len(generalLabels))
    for label in reversed(generalLabels):
        builder.PrependUOffsetTRelative(label)
    labelsOffset = builder.EndVector()

    Query.Start(builder)
    Query.AddProjectuuid(builder, projectUuidsOffset)
    Query.AddTimeinterval(builder, timeInterval)
    Query.AddLabel(builder, labelsOffset)
    Query.AddWithoutdata(builder, withoutData)

    return Query.End(builder)


def createTimeInterval(builder, timeMin, timeMax):
    '''Create a time time interval in flatbuffers'''
    TimeInterval.Start(builder)
    TimeInterval.AddTimeMin(builder, timeMin)
    TimeInterval.AddTimeMax(builder, timeMax)
    return TimeInterval.End(builder)
