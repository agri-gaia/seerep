import sys

from seerep.fb import (
    Boundingbox,
    BoundingBox2DLabeled,
    BoundingBox2DLabeledWithCategory,
    BoundingBoxes2DLabeledStamped,
    BoundingBoxLabeled,
    BoundingBoxLabeledWithCategory,
    BoundingboxStamped,
    CameraIntrinsics,
    CameraIntrinsicsQuery,
    Empty,
    GeodeticCoordinates,
    Header,
    Label,
    LabelsWithCategory,
    LabelWithInstance,
    Point,
    PointCloud2,
    PointField,
    ProjectCreation,
    ProjectInfo,
    ProjectInfos,
    Query,
    QueryInstance,
    RegionOfInterest,
    TimeInterval,
    Timestamp,
    TransformStampedQuery,
)
from seerep.fb import meta_operations_grpc_fb as metaOperations


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


def createProject(channel, builder, name, frameId, coordSys, ellipsoid, altitude, latitude, longitude):
    '''Create a project from the parameters'''
    stubMeta = metaOperations.MetaOperationsStub(channel)

    frameIdBuf = builder.CreateString(frameId)
    nameBuf = builder.CreateString(name)

    # create a geodetic coordinates object
    coordSysBuf = builder.CreateString(coordSys)
    ellipsoidBuf = builder.CreateString(ellipsoid)

    GeodeticCoordinates.Start(builder)
    GeodeticCoordinates.AddCoordinateSystem(builder, coordSysBuf)
    GeodeticCoordinates.AddEllipsoid(builder, ellipsoidBuf)
    GeodeticCoordinates.AddAltitude(builder, altitude)
    GeodeticCoordinates.AddLatitude(builder, latitude)
    GeodeticCoordinates.AddLongitude(builder, longitude)
    gc = GeodeticCoordinates.End(builder)

    ProjectCreation.Start(builder)
    ProjectCreation.AddMapFrameId(builder, frameIdBuf)
    ProjectCreation.AddName(builder, nameBuf)
    ProjectCreation.AddGeodeticPosition(builder, gc)
    projectCreationMsg = ProjectCreation.End(builder)
    builder.Finish(projectCreationMsg)

    buf = builder.Output()

    responseBuf = stubMeta.CreateProject(bytes(buf))
    response = ProjectInfo.ProjectInfo.GetRootAs(responseBuf)

    return response.Uuid().decode("utf-8")


def getOrCreateProject(
    builder,
    channel,
    name,
    create=True,
    mapFrameId="map",
    coordSys="",
    ellipsoid="",
    altitude=0.0,
    latitude=0.0,
    longitude=0.0,
):
    '''Get the project,, or if not present, create one'''
    projectUuid = getProject(builder, channel, name)

    if projectUuid is None:
        if create:
            projectUuid = createProject(
                channel, builder, name, mapFrameId, coordSys, ellipsoid, altitude, latitude, longitude
            )
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


def createLabelWithConfidence(builder, label, confidence=None):
    labelStr = builder.CreateString(label)
    Label.Start(builder)
    Label.AddLabel(builder, labelStr)
    if confidence:
        Label.AddConfidence(builder, confidence)
    return Label.End(builder)


def createLabelWithInstance(builder, label, confidence, instanceUuid):
    '''Creates a label with an associated instance uuid in flatbuffers'''
    labelConfidence = createLabelWithConfidence(builder, label, confidence)
    instanceUuidStr = builder.CreateString(instanceUuid)
    LabelWithInstance.Start(builder)
    LabelWithInstance.AddLabel(builder, labelConfidence)
    LabelWithInstance.AddInstanceUuid(builder, instanceUuidStr)
    return LabelWithInstance.End(builder)


# category: list of categories
# labels: list of list of labels (as fb-String msg) per category
def createLabelWithCategory(builder, category, labels):
    '''Creates the message representing the labels of a catogory'''
    LabelsCategories = []
    for iCategory in range(len(category)):
        categoryStr = builder.CreateString(category[iCategory])

        LabelsWithCategory.StartLabelsVector(builder, len(labels[iCategory]))
        for label in reversed(labels[iCategory]):
            builder.PrependUOffsetTRelative(label)
        labelsOffset = builder.EndVector()

        LabelsWithCategory.Start(builder)
        LabelsWithCategory.AddCategory(builder, categoryStr)
        LabelsWithCategory.AddLabels(builder, labelsOffset)
        LabelsCategories.append(LabelsWithCategory.End(builder))

    Query.StartLabelVector(builder, len(LabelsCategories))
    for LabelCategory in reversed(LabelsCategories):
        builder.PrependUOffsetTRelative(LabelCategory)
    return builder.EndVector()


def createLabelsWithInstance(builder, labels, confidences, instanceUuids):
    '''Creates multiple general labels'''
    assert len(labels) == len(instanceUuids)
    labelsGeneral = []
    for label, confidence, uuid in zip(labels, confidences, instanceUuids):
        labelsGeneral.append(createLabelWithInstance(builder, label, confidence, uuid))
    return labelsGeneral


def createPoint2d(builder, x, y):
    '''Creates a 2D point in flatbuffers'''
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    return Point.End(builder)


def createBoundingBox2d(builder, centerPoint, spatialExtent, rotation=0):
    '''Creates a 3D bounding box in flatbuffers'''
    Boundingbox.Start(builder)
    Boundingbox.AddCenterPoint(builder, centerPoint)
    Boundingbox.AddSpatialExtent(builder, spatialExtent)
    Boundingbox.AddRotation(builder, rotation)
    return Boundingbox.End(builder)


def createBoundingBox2dLabeled(builder, instance, boundingBox):
    '''Creates a labeled bounding box 2d in flatbuffers'''
    BoundingBox2DLabeled.Start(builder)
    BoundingBox2DLabeled.AddLabelWithInstance(builder, instance)
    BoundingBox2DLabeled.AddBoundingBox(builder, boundingBox)
    return BoundingBox2DLabeled.End(builder)


def createBoundingBoxes2d(builder, centerPoints, spatialExtents):
    assert len(centerPoints) == len(spatialExtents)
    boundingBoxes = []
    for centerPoint, spatialExtent in zip(centerPoints, spatialExtents):
        boundingBoxes.append(createBoundingBox2d(builder, centerPoint, spatialExtent))
    return boundingBoxes


def createBoundingBoxes2dLabeled(builder, instances, boundingBoxes):
    '''Creates multiple labeled bounding boxes'''
    assert len(instances) == len(boundingBoxes)
    boundingBoxes2dLabeled = []
    for instance, boundingBox in zip(instances, boundingBoxes):
        boundingBoxes2dLabeled.append(createBoundingBox2dLabeled(builder, instance, boundingBox))
    return boundingBoxes2dLabeled


def createBoundingBox2dLabeledStamped(builder, header, labelsBb):
    '''Creates a labeled bounding box 2d in flatbuffers'''
    BoundingBoxes2DLabeledStamped.StartLabelsBbVector(builder, len(labelsBb))
    for labelBb in reversed(labelsBb):
        builder.PrependUOffsetTRelative(labelBb)
    labelsBbVector = builder.EndVector()

    BoundingBoxes2DLabeledStamped.Start(builder)
    BoundingBoxes2DLabeledStamped.AddHeader(builder, header)
    BoundingBoxes2DLabeledStamped.AddLabelsBb(builder, labelsBbVector)
    return BoundingBoxes2DLabeledStamped.End(builder)


def createBoundingBox2DLabeledWithCategory(builder, category, bb2dLabeled):
    BoundingBox2DLabeledWithCategory.StartBoundingBox2dLabeledVector(builder, len(bb2dLabeled))
    for labelBb in reversed(bb2dLabeled):
        builder.PrependUOffsetTRelative(labelBb)
    labelsBbVector = builder.EndVector()

    BoundingBox2DLabeledWithCategory.Start(builder)
    BoundingBox2DLabeledWithCategory.AddCategory(builder, category)
    BoundingBox2DLabeledWithCategory.AddBoundingBox2dLabeled(builder, labelsBbVector)
    return BoundingBox2DLabeledWithCategory.End(builder)


def createPoint(builder, x, y, z):
    '''Creates a 3D point in flatbuffers'''
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    Point.AddZ(builder, z)
    return Point.End(builder)


def createBoundingBox(builder, centerPoint, spatialExtent, rotation=None):
    '''Creates a 3D bounding box in flatbuffers'''
    Boundingbox.Start(builder)
    Boundingbox.AddCenterPoint(builder, centerPoint)
    Boundingbox.AddSpatialExtent(builder, spatialExtent)
    if rotation:
        Boundingbox.AddRotation(builder, rotation)
    return Boundingbox.End(builder)


def createBoundingBoxStamped(builder, header, centerPoint, spatialExtent, rotation=None):
    '''Creates a stamped 3D bounding box in flatbuffers'''
    boundingBox = createBoundingBox(builder, centerPoint, spatialExtent, rotation)
    BoundingboxStamped.Start(builder)
    BoundingboxStamped.AddHeader(builder, header)
    BoundingboxStamped.AddBoundingbox(builder, boundingBox)
    return BoundingboxStamped.End(builder)


def createBoundingBoxes(builder, centerPoint, spatialExtent, rotation=None):
    assert len(centerPoint) == len(spatialExtent)
    boundingBoxes = []
    if rotation:
        for center, extent, rot in zip(centerPoint, spatialExtent, rotation):
            boundingBoxes.append(createBoundingBox(builder, center, extent, rot))
    else:
        for center, extent in zip(centerPoint, spatialExtent):
            boundingBoxes.append(createBoundingBox(builder, center, extent))
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


def createBoundingBoxLabeledWithCategory(builder, category, bbLabeled):
    BoundingBoxLabeledWithCategory.StartBoundingBoxLabeledVector(builder, len(bbLabeled))
    for labelBb in reversed(bbLabeled):
        builder.PrependUOffsetTRelative(labelBb)
    labelsBbVector = builder.EndVector()

    BoundingBoxLabeledWithCategory.Start(builder)
    BoundingBoxLabeledWithCategory.AddCategory(builder, category)
    BoundingBoxLabeledWithCategory.AddBoundingBox2dLabeled(builder, labelsBbVector)
    return BoundingBoxLabeledWithCategory.End(builder)


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
        Query.AddLabel(builder, labels)
    # no if; has default value
    Query.AddMustHaveAllLabels(builder, mustHaveAllLabels)
    if projectUuids:
        Query.AddProjectuuid(builder, projectUuidsOffset)
    if instanceUuids:
        Query.AddInstanceuuid(builder, instanceOffset)
    if dataUuids:
        Query.QueryAddDatauuid(builder, dataUuidOffset)
    # no if; has default value
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


def createRegionOfInterest(builder, x_offset, y_offset, height, width, do_rectify):
    RegionOfInterest.Start(builder)
    RegionOfInterest.AddXOffset(builder, x_offset)
    RegionOfInterest.AddYOffset(builder, y_offset)
    RegionOfInterest.AddHeight(builder, height)
    RegionOfInterest.AddWidth(builder, width)
    RegionOfInterest.AddDoRectify(builder, do_rectify)
    return RegionOfInterest.End(builder)


def createCameraIntrinsics(
    builder,
    header,
    height,
    width,
    distortion_model,
    distortion,
    intrinsics_matrix,
    rectification_matrix,
    projection_matrix,
    binning_x,
    binning_y,
    region_of_interest,
):
    dm_buf = builder.CreateString(distortion_model)

    CameraIntrinsics.StartDistortionVector(builder, len(distortion))
    for d in reversed(distortion):
        builder.PrependFloat64(d)
    distortionOffset = builder.EndVector()

    CameraIntrinsics.StartIntrinsicMatrixVector(builder, len(intrinsics_matrix))
    for im in reversed(intrinsics_matrix):
        builder.PrependFloat64(im)
    IMOffset = builder.EndVector()

    CameraIntrinsics.StartRectificationMatrixVector(builder, len(rectification_matrix))
    for rm in reversed(rectification_matrix):
        builder.PrependFloat64(rm)
    RMOffset = builder.EndVector()

    CameraIntrinsics.StartProjectionMatrixVector(builder, len(projection_matrix))
    for pm in reversed(projection_matrix):
        builder.PrependFloat64(pm)
    PMOffset = builder.EndVector()

    CameraIntrinsics.Start(builder)
    CameraIntrinsics.AddHeader(builder, header)
    CameraIntrinsics.AddHeight(builder, height)
    CameraIntrinsics.AddWidth(builder, width)
    CameraIntrinsics.AddDistortionModel(builder, dm_buf)
    CameraIntrinsics.AddDistortion(builder, distortionOffset)
    CameraIntrinsics.AddIntrinsicMatrix(builder, IMOffset)
    CameraIntrinsics.AddRectificationMatrix(builder, RMOffset)
    CameraIntrinsics.AddProjectionMatrix(builder, PMOffset)
    CameraIntrinsics.AddBinningX(builder, binning_x)
    CameraIntrinsics.AddBinningY(builder, binning_y)
    CameraIntrinsics.AddRegionOfInterest(builder, region_of_interest)

    return CameraIntrinsics.End(builder)


def createCameraIntrinsicsQuery(builder, ci_uuid, project_uuid):
    ci_uuid_str = builder.CreateString(ci_uuid)
    project_uuid_str = builder.CreateString(project_uuid)
    CameraIntrinsicsQuery.Start(builder)
    CameraIntrinsicsQuery.AddUuidCameraIntrinsics(builder, ci_uuid_str)
    CameraIntrinsicsQuery.AddUuidProject(builder, project_uuid_str)

    return CameraIntrinsicsQuery.End(builder)
