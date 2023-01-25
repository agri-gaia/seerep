import sys

from fb import (
    Boundingbox,
    BoundingBox2DLabeled,
    BoundingBoxes2DLabeledStamped,
    BoundingBoxLabeled,
    BoundingboxStamped,
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
    QueryInstance,
    TimeInterval,
    Timestamp,
    TransformStampedQuery,
    UuidDatatypePair,
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


def createTimeStamp(builder, seconds, nanoseconds=0):
    '''Create a time stamp in flatbuffers'''
    Timestamp.Start(builder)
    Timestamp.AddSeconds(builder, seconds)
    Timestamp.AddNanos(builder, nanoseconds)
    return Timestamp.End(builder)


def createHeader(builder, timeStamp=None, frame=None, projectUuid=None, msgUuid=None):
    '''Creates a message header in flatbuffers, all parameters are optional'''
    if frame:
        frameStr = builder.CreateString(frame)
    if projectUuid:
        projectUuidStr = builder.CreateString(projectUuid)
    if msgUuid:
        msgUuidStr = builder.CreateString(msgUuid)
    Header.Start(builder)
    if frame:
        Header.AddFrameId(builder, frameStr)
    if timeStamp:
        Header.AddStamp(builder, timeStamp)
    if projectUuid:
        Header.AddUuidProject(builder, projectUuidStr)
    if msgUuid:
        Header.AddUuidMsgs(builder, msgUuidStr)
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


def createPoint2d(builder, x, y):
    '''Creates a 2D point in flatbuffers'''
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    return Point.End(builder)


def createBoundingBox2d(builder, point2dMin, point2dMax):
    '''Creates a 3D bounding box in flatbuffers'''
    Boundingbox.Start(builder)
    Boundingbox.AddPointMin(builder, point2dMin)
    Boundingbox.AddPointMax(builder, point2dMax)
    return Boundingbox.End(builder)


def createBoundingBox2dLabeled(builder, instance, boundingBox):
    '''Creates a labeled bounding box 2d in flatbuffers'''
    BoundingBox2DLabeled.Start(builder)
    BoundingBox2DLabeled.AddLabelWithInstance(builder, instance)
    BoundingBox2DLabeled.AddBoundingBox(builder, boundingBox)
    return BoundingBox2DLabeled.End(builder)


def createBoundingBoxes2d(builder, minPoints, maxPoints):
    assert len(minPoints) == len(maxPoints)
    boundingBoxes = []
    for pointMin, pointMax in zip(minPoints, maxPoints):
        boundingBoxes.append(createBoundingBox2d(builder, pointMin, pointMax))
    return boundingBoxes


def createBoundingBoxes2dLabeled(builder, instances, boundingBoxes):
    '''Creates multiple labeled bounding boxes'''
    assert len(instances) == len(boundingBoxes)
    boundingBoxes2dLabeled = []
    for instance, boundingBox in zip(instances, boundingBoxes):
        boundingBoxes2dLabeled.append(createBoundingBox2dLabeled(builder, instance, boundingBox))
    return boundingBoxes2dLabeled


def createBoundingBox2dLabeledStamped(builder, header, boundingBox2dLabeledVector):
    '''Creates a labeled bounding box 2d in flatbuffers'''
    BoundingBoxes2DLabeledStamped.Start(builder)
    BoundingBoxes2DLabeledStamped.AddHeader(builder, header)
    BoundingBoxes2DLabeledStamped.AddLabelsBb(builder, boundingBox2dLabeledVector)
    return BoundingBoxes2DLabeledStamped.End(builder)


def createPoint(builder, x, y, z):
    '''Creates a 3D point in flatbuffers'''
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    Point.AddZ(builder, z)
    return Point.End(builder)


def createBoundingBox(builder, pointMin, pointMax):
    '''Creates a 3D bounding box in flatbuffers'''
    Boundingbox.Start(builder)
    Boundingbox.AddPointMin(builder, pointMin)
    Boundingbox.AddPointMax(builder, pointMax)
    return Boundingbox.End(builder)


def createBoundingBoxStamped(builder, header, pointMin, pointMax):
    '''Creates a stamped 3D bounding box in flatbuffers'''
    boundingBox = createBoundingBox(builder, pointMin, pointMax)
    BoundingboxStamped.Start(builder)
    BoundingboxStamped.AddHeader(builder, header)
    BoundingboxStamped.AddBoundingbox(builder, boundingBox)
    return BoundingboxStamped.End(builder)


def createBoundingBoxes(builder, minPoints, maxPoints):
    assert len(minPoints) == len(maxPoints)
    boundingBoxes = []
    for pointMin, pointMax in zip(minPoints, maxPoints):
        boundingBoxes.append(createBoundingBox(builder, pointMin, pointMax))
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


def createQuery(
    builder,
    boundingBox=None,
    timeInterval=None,
    labels=None,
    mustHaveAllLabels=False,
    projectUuids=None,
    instanceUuids=None,
    dataUuids=None,
    withoutData=False,
):
    '''Create a query, all parameters are optional'''

    # Note: reverse because we prepend
    if projectUuids:
        Query.StartProjectuuidVector(builder, len(projectUuids))
        for projectUuid in reversed(projectUuids):
            builder.PrependUOffsetTRelative(projectUuid)
        projectUuidsOffset = builder.EndVector()

    if labels:
        Query.StartLabelVector(builder, len(labels))
        for label in reversed(labels):
            builder.PrependUOffsetTRelative(label)
        labelsOffset = builder.EndVector()

    if instanceUuids:
        Query.StartInstanceuuidVector(builder, len(instanceUuids))
        for instance in reversed(instanceUuids):
            builder.PrependUOffsetTRelative(instance)
        instanceOffset = builder.EndVector()

    if dataUuids:
        Query.StartDatauuidVector(builder, len(dataUuids))
        for dataUuid in reversed(dataUuids):
            builder.PrependUOffsetTRelative(dataUuid)
        dataUuidOffset = builder.EndVector()

    Query.Start(builder)
    if boundingBox:
        Query.AddBoundingboxStamped(builder, boundingBox)
    if timeInterval:
        Query.AddTimeinterval(builder, timeInterval)
    if labels:
        Query.AddLabel(builder, labelsOffset)
    Query.AddMustHaveAllLabels(builder, mustHaveAllLabels)
    if projectUuids:
        Query.AddProjectuuid(builder, projectUuidsOffset)
    if instanceUuids:
        Query.AddInstanceuuid(builder, instanceOffset)
    if dataUuids:
        Query.QueryAddDatauuid(builder, dataUuidOffset)
    Query.AddWithoutdata(builder, withoutData)

    return Query.End(builder)


def createQueryInstance(builder, query, datatype):
    '''Create a query for instances'''
    QueryInstance.Start(builder)
    QueryInstance.AddDatatype(builder, datatype)
    QueryInstance.AddQuery(builder, query)
    return QueryInstance.End(builder)


def createTimeInterval(builder, timeMin, timeMax):
    '''Create a time time interval in flatbuffers'''
    TimeInterval.Start(builder)
    TimeInterval.AddTimeMin(builder, timeMin)
    TimeInterval.AddTimeMax(builder, timeMax)
    return TimeInterval.End(builder)


def createTransformStampedQuery(builder, header, childFrameId):
    TransformStampedQuery.Start(builder)
    TransformStampedQuery.AddHeader(builder, header)
    TransformStampedQuery.AddChildFrameId(builder, childFrameId)
    return TransformStampedQuery.End(builder)


def createUuidDatatypePair(builder, uuid, datatype):
    uuidStr = builder.CreateString(uuid)

    UuidDatatypePair.Start(builder)
    UuidDatatypePair.AddProjectuuid(builder, uuidStr)
    UuidDatatypePair.AddDatatype(builder, datatype)
    return UuidDatatypePair.End(builder)


def createProjectInfo(builder, name, uuid):
    nameStr = builder.CreateString(name)
    uuidStr = builder.CreateString(uuid)

    ProjectInfo.Start(builder)
    ProjectInfo.AddName(builder, nameStr)
    ProjectInfo.AddUuid(builder, uuidStr)
    return ProjectInfo.End(builder)
