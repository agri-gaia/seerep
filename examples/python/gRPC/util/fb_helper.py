import sys
from typing import List

import numpy as np
from seerep.fb import (
    Boundingbox,
    CameraIntrinsics,
    CameraIntrinsicsQuery,
    Empty,
    GeodeticCoordinates,
    Header,
    Image,
    Label,
    LabelCategory,
    Point,
    PointCloud2,
    PointField,
    PointStamped,
    Polygon2D,
    ProjectCreation,
    ProjectInfo,
    ProjectInfos,
    Quaternion,
    Query,
    QueryInstance,
    RegionOfInterest,
    TimeInterval,
    Timestamp,
    TransformStamped,
    TransformStampedQuery,
    UuidDatatypePair,
    UuidDatatypeWithCategory,
    Vector3,
)
from seerep.fb import meta_operations_grpc_fb as metaOperations


# TODO: should be moved into a separate module
def rosToNumpyDtype(ros_dtype: int) -> np.dtype:
    """Converts the numeric represenations of dtypes in ROS to numpy dtype objects."""
    if ros_dtype == 1:
        return np.dtype(np.int8)
    elif ros_dtype == 2:
        return np.dtype(np.uint8)
    elif ros_dtype == 3:
        return np.dtype(np.int16)
    elif ros_dtype == 4:
        return np.dtype(np.uint16)
    elif ros_dtype == 5:
        return np.dtype(np.int32)
    elif ros_dtype == 6:
        return np.dtype(np.uint32)
    elif ros_dtype == 7:
        return np.dtype(np.float32)
    elif ros_dtype == 8:
        return np.dtype(np.float64)
    else:
        raise ValueError("Unknown dtype")


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


def getProjectInfo(builder, channel, name):
    '''Retrieve project infos by name'''
    stubMeta = metaOperations.MetaOperationsStub(channel)

    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stubMeta.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    for i in range(response.ProjectsLength()):
        if response.Projects(i).Name().decode("utf-8") == name:
            return {
                "name": response.Projects(i).Name().decode("utf-8"),
                "uuid": response.Projects(i).Uuid().decode("utf-8"),
                "frameid": response.Projects(i).Frameid().decode("utf-8"),
                "geodetic_position": response.Projects(i).GeodeticPosition(),
            }

    return None


def createProject(channel, builder, name, frameId, coordSys, altitude, latitude, longitude):
    '''Create a project from the parameters'''
    stubMeta = metaOperations.MetaOperationsStub(channel)

    frameIdBuf = builder.CreateString(frameId)
    nameBuf = builder.CreateString(name)

    # create a geodetic coordinates object
    coordSysBuf = builder.CreateString(coordSys)

    GeodeticCoordinates.Start(builder)
    GeodeticCoordinates.AddCoordinateSystem(builder, coordSysBuf)
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
    altitude=0.0,
    latitude=0.0,
    longitude=0.0,
):
    '''Get the project,, or if not present, create one'''
    projectUuid = getProject(builder, channel, name)

    if projectUuid is None:
        if create:
            projectUuid = createProject(channel, builder, name, mapFrameId, coordSys, altitude, latitude, longitude)
        else:
            sys.exit()

    return projectUuid


def createEmpty(builder):
    '''Create an empty flatbuffer'''
    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    return builder.Output()


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


def createPoint2d(builder, x, y):
    '''Creates a 2D point in flatbuffers'''
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    return Point.End(builder)


def createPoint(builder, x, y, z):
    '''Creates a 3D point in flatbuffers'''
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    Point.AddZ(builder, z)
    return Point.End(builder)


def createPointStamped(builder, point, header, labelGeneralCategoryVector):
    '''Creates a 3D point stamped in flatbuffers'''

    PointStamped.Start(builder)
    PointStamped.AddPoint(builder, point)
    PointStamped.AddHeader(builder, header)
    PointStamped.AddLabelsGeneral(builder, labelGeneralCategoryVector)
    return PointStamped.End(builder)


def createBoundingBox(builder, centerPoint, spatialExtent, rotation=None):
    '''Creates a 3D bounding box in flatbuffers'''
    Boundingbox.Start(builder)
    Boundingbox.AddCenterPoint(builder, centerPoint)
    Boundingbox.AddSpatialExtent(builder, spatialExtent)
    if rotation:
        Boundingbox.AddRotation(builder, rotation)
    return Boundingbox.End(builder)


def createPolygon2D(builder, height, z, vertices):
    '''Create a 2D Polygon in flatbuffers'''

    Polygon2D.StartVerticesVector(builder, len(vertices))
    for v in vertices:
        builder.PrependUOffsetTRelative(v)
    vertices_fb = builder.EndVector()

    Polygon2D.Start(builder)
    Polygon2D.AddHeight(builder, height)
    Polygon2D.AddZ(builder, z)
    Polygon2D.AddVertices(builder, vertices_fb)

    return Polygon2D.End(builder)


def addToBoundingBoxLabeledVector(builder, boundingBoxLabeledList):
    '''Adds list of boudingBoxLabeled into the labelsBbVector of a flatbuffers pointcloud2'''
    PointCloud2.StartLabelsBbVector(builder, len(boundingBoxLabeledList))
    for bb in reversed(boundingBoxLabeledList):
        builder.PrependUOffsetTRelative(bb)
    return builder.EndVector()


def addToPointFieldVector(builder, pointFieldList):
    '''Adds a list of pointFields into the fieldsVector of a flatbuffers pointcloud2'''
    PointCloud2.StartFieldsVector(builder, len(pointFieldList))
    for pointField in reversed(pointFieldList):
        builder.PrependUOffsetTRelative(pointField)
    return builder.EndVector()


def createQuery(
    builder,
    timeInterval=None,
    labels=None,
    mustHaveAllLabels=False,
    projectUuids=None,
    instanceUuids=None,
    dataUuids=None,
    withoutData=False,
    polygon2d=None,
    fullyEncapsulated=False,
    inMapFrame=True,
    sortByTime=False,
):
    '''Create a query, all parameters are optional'''

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
    if polygon2d:
        Query.AddPolygon(builder, polygon2d)
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
    Query.AddFullyEncapsulated(builder, fullyEncapsulated)
    Query.AddInMapFrame(builder, inMapFrame)
    Query.AddSortByTime(builder, sortByTime)

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
    max_viewing_dist,
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
    CameraIntrinsics.AddMaximumViewingDistance(builder, max_viewing_dist)

    return CameraIntrinsics.End(builder)


def createCameraIntrinsicsQuery(builder, ci_uuid, project_uuid):
    ci_uuid_str = builder.CreateString(ci_uuid)
    project_uuid_str = builder.CreateString(project_uuid)
    CameraIntrinsicsQuery.Start(builder)
    CameraIntrinsicsQuery.AddUuidCameraIntrinsics(builder, ci_uuid_str)
    CameraIntrinsicsQuery.AddUuidProject(builder, project_uuid_str)

    return CameraIntrinsicsQuery.End(builder)


def createUuidDatatypePair(builder, uuid, datatype):
    uuidStr = builder.CreateString(uuid)

    UuidDatatypePair.Start(builder)
    UuidDatatypePair.AddProjectuuid(builder, uuidStr)
    UuidDatatypePair.AddDatatype(builder, datatype)
    return UuidDatatypePair.End(builder)


def createUuidDatatypeWithCategory(builder, uuid, datatype, category):
    categoryStr = builder.CreateString(category)

    UuidDatatypePair = createUuidDatatypePair(builder, uuid, datatype)

    UuidDatatypeWithCategory.Start(builder)
    UuidDatatypeWithCategory.AddCategory(builder, categoryStr)
    UuidDatatypeWithCategory.AddUuidAndDatatype(builder, UuidDatatypePair)
    return UuidDatatypeWithCategory.End(builder)


def createProjectInfo(builder, name, uuid):
    nameStr = builder.CreateString(name)
    uuidStr = builder.CreateString(uuid)

    ProjectInfo.Start(builder)
    ProjectInfo.AddName(builder, nameStr)
    ProjectInfo.AddUuid(builder, uuidStr)
    return ProjectInfo.End(builder)


def createVector3(builder, t):
    Vector3.Start(builder)
    Vector3.AddX(builder, t[0])
    Vector3.AddY(builder, t[1])
    Vector3.AddZ(builder, t[2])
    return Vector3.End(builder)


def createQuaternion(builder, quat):
    Quaternion.Start(builder)
    Quaternion.AddX(builder, quat.x)
    Quaternion.AddY(builder, quat.y)
    Quaternion.AddZ(builder, quat.z)
    Quaternion.AddW(builder, quat.w)
    return Quaternion.End(builder)


def createTransformStamped(builder, childFrame, headerTf, transform):
    childFrame = builder.CreateString(childFrame)
    TransformStamped.Start(builder)
    TransformStamped.AddChildFrameId(builder, childFrame)
    TransformStamped.AddHeader(builder, headerTf)
    TransformStamped.AddTransform(builder, transform)
    return TransformStamped.End(builder)


def create_image(
    builder,
    header: Header.Header,
    encoding: str,
    is_bigendian: bool,
    image: np.ndarray,
    camera_intrinsics_uuid: str,
    labels: LabelCategory.LabelCategory = None,
):
    encoding_offset = builder.CreateString(encoding)
    camera_intrinsics_uuid_offset = builder.CreateString(camera_intrinsics_uuid)
    data_offset = builder.CreateByteVector(image.tobytes())

    Image.Start(builder)
    Image.AddHeader(builder, header)
    Image.AddHeight(builder, image.shape[0])
    Image.AddWidth(builder, image.shape[1])
    Image.AddEncoding(builder, encoding_offset)
    Image.AddIsBigendian(builder, is_bigendian)
    Image.AddStep(builder, image.nbytes // image.shape[0])
    Image.AddData(builder, data_offset)
    if labels:
        Image.AddLabels(builder, labels)
    Image.AddUuidCameraintrinsics(builder, camera_intrinsics_uuid_offset)
    return Image.End(builder)


# TODO: find a way to not use to ifs for the optional parameters
def create_label(builder, label: str, label_id: int, instance_uuid: str = None, instance_id: int = None):
    label_offset = builder.CreateString(label)
    if instance_uuid:
        instance_uuid_offset = builder.CreateString(instance_uuid)
    Label.Start(builder)
    Label.AddLabel(builder, label_offset)
    Label.LabelAddLabelIdDatumaro(builder, label_id)
    if instance_uuid:
        Label.AddInstanceUuid(builder, instance_uuid_offset)
    if instance_id:
        Label.AddInstanceId(builder, instance_id)
    return Label.End(builder)


def create_label_category(builder, labels: List[Label.Label], datumaro_json: str, category: str = None):
    if category:
        category_offset = builder.CreateString(category)
    datumaro_json_offset = builder.CreateString(datumaro_json)

    LabelCategory.StartLabelsVector(builder, len(labels))
    for label in labels:
        builder.PrependUOffsetTRelative(label)
    vec_offset = builder.EndVector()

    LabelCategory.Start(builder)
    if category:
        LabelCategory.AddCategory(builder, category_offset)
    LabelCategory.AddLabels(builder, vec_offset)
    LabelCategory.AddDatumaroJson(builder, datumaro_json_offset)
    return LabelCategory.End(builder)
