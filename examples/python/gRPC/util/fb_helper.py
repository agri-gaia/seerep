import sys
from typing import Dict, List, Tuple, Union

import numpy as np
from flatbuffers import Builder
from grpc import Channel
from seerep.fb import (
    Boundingbox,
    CameraIntrinsics,
    CameraIntrinsicsQuery,
    Empty,
    GeodeticCoordinates,
    Header,
    Image,
    Label,
    LabelsWithCategory,
    LabelsWithInstanceWithCategory,
    LabelWithInstance,
    Point,
    Point2D,
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
    Transform,
    TransformStamped,
    TransformStampedQuery,
    UuidDatatypePair,
    UuidDatatypeWithCategory,
    Vector3,
)
from seerep.fb import meta_operations_grpc_fb as metaOperations


# ruff: noqa: PLR0911
# TODO: should be moved into a separate module
def rosToNumpyDtype(ros_dtype: int) -> np.dtype:
    """
    Converts the numeric represenations of dtypes in ROS to numpy dtype objects.

    Args:
        ros_dtype: The numeric representation of the dtype in ROS\
            (see [ROS documentation](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointField.html))

    Returns:
        The numpy dtype equivalent
    """
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


def getProject(builder: Builder, channel: Channel, name: str) -> Union[str, None]:
    """
    Retrieve a project by name.

    Args:
        builder: A flatbuffers Builder
        channel: The gRPC channel
        name: The name of the project

    Returns:
        The UUID of the project if found, None otherwise
    """
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


def getProjectInfo(builder: Builder, channel: Channel, name: str) -> Union[Dict[str, str], None]:
    """
    Retrieve project infos by name.

    Args:
        builder: A flatbuffers Builder
        channel: The gRPC channel
        name: The name of the project

    Returns:
        A dictionary containing the project information or None
    """
    stubMeta = metaOperations.MetaOperationsStub(channel)

    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stubMeta.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    for i in range(response.ProjectsLength()):
        if response.Projects(i).Name().decode("utf-8") == name:
            project = response.Projects(i)
            return {
                "name": project.Name().decode("utf-8"),
                "uuid": project.Uuid().decode("utf-8"),
                "frameid": project.Frameid().decode("utf-8"),
                "geodetic_position": f"lat: {project.GeodeticPosition().Latitude()},\
                    long: {project.GeodeticPosition().Longitude()},\
                    alt: {project.GeodeticPosition().Altitude()}",
                "coordinate_system": project.GeodeticPosition().CoordinateSystem().decode("utf-8"),
            }

    return None


def createProjectRaw(
    channel: Channel,
    builder: Builder,
    name: str,
    frameId: str,
    coordSys: str,
    altitude: float,
    latitude: float,
    longitude: float,
) -> bytearray:
    """
    Create a project from the parameters and return it as a flatbuffers object.

    Args:
        channel: The gRPC channel
        builder: A flatbuffers Builder
        name: The name of the project
        frameId: The coordinate frame of the project
        coordSys: The coordinate system type as a [proj ellipsoid](https://proj.org/en/stable/usage/ellipsoids.html#built-in-ellipsoid-definitions)\
            code
        altitude: The altitude of the projects position on the globe (according to coordSys)
        latitude: The latitude of the projects position on the globe (according to coordSys)
        longitude: The longitude of the project positition on the globe (according to coordSys)

    Returns:
        A flatbuffers object of type ProjectInfo representing the project
    """
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
    return responseBuf


def createProject(
    channel: Channel,
    builder: Builder,
    name: str,
    frameId: str,
    coordSys: str,
    altitude: float,
    latitude: float,
    longitude: float,
) -> str:
    """
    Create a project from the parameters.

    Args:
        channel: The gRPC channel
        builder: A flatbuffers Builder
        name: The name of the project
        frameId: The coordinate frame of the project
        coordSys: The coordinate system type as a [proj ellipsoid](https://proj.org/en/stable/usage/ellipsoids.html#built-in-ellipsoid-definitions)\
            code
        altitude: The altitude of the projects position on the globe (according to coordSys)
        latitude: The latitude of the projects position on the globe (according to coordSys)
        longitude: The longitude of the project positition on the globe (according to coordSys)

    Returns:
        The UUID of the created project
    """
    return (
        ProjectInfo.ProjectInfo.GetRootAs(
            createProjectRaw(
                channel,
                builder,
                name,
                frameId,
                coordSys,
                altitude,
                latitude,
                longitude,
            )
        )
        .Uuid()
        .decode("utf-8")
    )


def getOrCreateProject(
    builder: Builder,
    channel: Channel,
    name: str,
    create: bool = True,
    mapFrameId: str = "map",
    coordSys: str = "",
    altitude: float = 0.0,
    latitude: float = 0.0,
    longitude: float = 0.0,
) -> str:
    """
    Get the project, or if not present, create one.

    Args:
        builder: A flatbuffers Builder
        channel: The gRPC channel
        name: The name of the project
        create: Whether to create the project if it does not exist
        mapFrameId: The coordinate frame of the project
        coordSys: The coordinate system type as a [proj ellipsoid](https://proj.org/en/stable/usage/ellipsoids.html#built-in-ellipsoid-definitions)\
            code
        altitude: The altitude of the projects position on the globe (according to coordSys)
        latitude: The latitude of the project position on the globe (according to coordSys)
        longitude: The longitude of the project position on the globe (according to coordSys)

    Returns:
        The UUID of the project
    """
    projectUuid = getProject(builder, channel, name)

    if projectUuid is None:
        if create:
            projectUuid = createProject(
                channel,
                builder,
                name,
                mapFrameId,
                coordSys,
                altitude,
                latitude,
                longitude,
            )
        else:
            sys.exit()

    return projectUuid


def createEmpty(builder: Builder) -> bytes:
    """
    Create an empty flatbuffer

    Args:
        builder: A flatbuffers Builder

    Returns:
        A bytearray representing the empty flatbuffer
    """
    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    return bytes(builder.Output())


def deleteProject(channel: Channel, builder: Builder, projectName: str, projectUuid: str):
    """
    Delete a project.

    Args:
        channel: The gRPC channel
        builder: A flatbuffers Builder
        projectUuid: The UUID of the project to delete
    """
    stub = metaOperations.MetaOperationsStub(channel)
    projInfoMsg = createProjectInfo(builder, projectName, projectUuid)
    builder.Finish(projInfoMsg)
    buf = builder.Output()
    stub.DeleteProject(bytes(buf))


def createTimeStamp(builder: Builder, seconds: int, nanoseconds: int = 0) -> int:
    """
    Create a time stamp in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        seconds: The seconds since epoch
        nanoseconds: The nanoseconds since the last second

    Returns:
        A pointer to the constructed timestamp object
    """
    Timestamp.Start(builder)
    Timestamp.AddSeconds(builder, seconds)
    Timestamp.AddNanos(builder, nanoseconds)
    return Timestamp.End(builder)


def createHeader(
    builder: Builder, timeStamp: int = None, frame: str = None, projectUuid: str = None, msgUuid: str = None
) -> int:
    """
    Creates a message header in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        timeStamp: The pointer to a constructed Timestamp object
        frame: The coordinate frame of the message
        projectUuid: The UUID of the project the message belongs to
        msgUuid: The UUID of the message (when not provided it will be set by the server)

    Returns:
        A pointer to the constructed header object
    """
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
def createPointField(builder: Builder, name: str, offset: int, datatype: int, count: int) -> int:
    """
    Creates a point field to describe the structure of a point entry in the Pointcloud2 message.
    This esssentially mimicks the ROS\
    [Pointfield](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointField.html)\
    message.

    Args:
        builder: A flatbuffers Builder
        name: The name of the point field
        offset: Offset from start of the point struct
        datatype: Datatype used for the point entries\
            (see [fbs definition](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_field.fbs))
        count: Number of elements in the point field

    Returns:
        A pointer to the constructed point field object
    """
    nameStr = builder.CreateString(name)
    PointField.Start(builder)
    PointField.AddName(builder, nameStr)
    PointField.AddOffset(builder, offset)
    PointField.AddDatatype(builder, datatype)
    PointField.AddCount(builder, count)
    return PointField.End(builder)


def createPointFields(
    builder: Builder,
    channels: List[str],
    datatype: int,
    dataTypeOffset: int,
    count: int,
) -> List[int]:
    """
    Creates point fields for all specified channels.

    Args:
        builder: A flatbuffers Builder
        channels: List of channel names
        datatype: Datatype used for the point entries\
            (see [fbs definition](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_field.fbs))
        dataTypeOffset: Offset from start of the point struct, in this case the offset is used for every channel
        count: Number of elements in each point field

    Returns:
        A list of pointers to the constructed point field objects
    """
    pointFieldsList = []
    offset = 0
    for channel in channels:
        pointFieldsList.append(createPointField(builder, channel, offset, datatype, count))
        offset += dataTypeOffset
    return pointFieldsList


def createLabelWithConfidence(builder: Builder, label: str, confidence: Union[float, None] = None) -> int:
    """
    Creates a label with an associated confidence value of type\
    [Label](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/label.fbs).

    Args:
        builder: A flatbuffers Builder
        label: The label string
        confidence: The confidence value of the label

    Returns:
        A pointer to the constructed label object
    """

    labelStr = builder.CreateString(label)
    Label.Start(builder)
    Label.AddLabel(builder, labelStr)
    if confidence:
        Label.AddConfidence(builder, confidence)
    return Label.End(builder)


def createLabelsWithConfidences(
    builder: Builder, labels: List[str], confidences: Union[List[float], None] = None
) -> List[int]:
    """
    Creates multiple labels with associated confidence values of type\
    [Label](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/label.fbs).

    Args:
        builder: A flatbuffers Builder
        labels: A list of label strings
        confidences: A list of confidence values corresponding to the label on the same index

    Returns:
        A list of pointers to the constructed label objects
    """

    labels_with_confidences = []
    if confidences:
        for label, confidence in zip(labels, confidences):
            labels_with_confidences.append(createLabelWithConfidence(builder, label, confidence))
    else:
        for label in labels:
            labels_with_confidences.append(createLabelWithConfidence(builder, label))

    return labels_with_confidences


def createLabelWithInstance(builder: str, label: str, instanceUuid: str, confidence: Union[float, None] = None) -> int:
    """
    Creates a label with an associated instance uuid of type\
    [LabelWithInstance](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/label_with_instance.fbs).

    Args:
        builder: A flatbuffers Builder
        label: The label string
        confidence: The confidence value of the label
        instanceUuid: The UUID of the instance which the label belongs to

    Returns:
        A pointer to the constructed label object
    """

    labelConfidence = createLabelWithConfidence(builder, label, confidence)
    instanceUuidStr = builder.CreateString(instanceUuid)
    LabelWithInstance.Start(builder)
    LabelWithInstance.AddLabel(builder, labelConfidence)
    LabelWithInstance.AddInstanceUuid(builder, instanceUuidStr)
    return LabelWithInstance.End(builder)


# category: list of categories
# labels: list of list of labels (as fb-String msg) per category
def createLabelsWithCategoryVector(
    builder: Builder,
    categories: List[int],
    cat_labels: List[List[str]],
    cat_confidences: Union[List[List[float]], None] = None,
) -> int:
    """
    Adds the same list of labels to each category of a list of categories.

    Args:
        builder: A flatbuffers Builder
        categories: A list with categories
        labels: A inner list of labels for each category the outer list corresponding to the categories
        confidences: A inner list of confidence values for each category with the outer\
            list corresponding to the categories

    Returns:
        A pointer to the vector of category to labels mappings
    """

    LabelsCategories = createLabelsWithCategories(builder, categories, cat_labels, cat_confidences)

    Query.StartLabelVector(builder, len(LabelsCategories))
    for LabelCategory in reversed(LabelsCategories):
        builder.PrependUOffsetTRelative(LabelCategory)
    return builder.EndVector()


def createLabelsWithCategories(
    builder: Builder,
    categories: List[int],
    cat_labels: List[List[str]],
    cat_confidences: Union[List[List[float]], None] = None,
) -> List[int]:
    """
    Adds the same list of labels to each category of a list of categories.

    Args:
        builder: A flatbuffers Builder
        categories: A list with categories
        labels: A inner list of labels for each category the outer list corresponding to the categories
        confidences: A inner list of confidence values for each category with the outer\
            list corresponding to the categories

    Returns:
        A list to pointers to\
        [LabelsWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/labels_with_category.fbs)\
        objects
    """
    label_categories = []

    assert len(cat_labels) == len(categories)

    for triple in zip(categories, cat_labels, cat_confidences) if cat_confidences else zip(categories, cat_labels):
        cat_str = builder.CreateString(str(triple[0]))

        if len(triple) > 2:
            labels_processed = createLabelsWithConfidences(builder, triple[1], triple[2])
        else:
            labels_processed = createLabelsWithConfidences(builder, triple[1])

        LabelsWithCategory.StartLabelsVector(builder, len(labels_processed))
        for label in reversed(labels_processed):
            builder.PrependUOffsetTRelative(label)
        labels_offset = builder.EndVector()

        LabelsWithCategory.Start(builder)
        LabelsWithCategory.AddCategory(builder, cat_str)
        LabelsWithCategory.AddLabels(builder, labels_offset)
        label_categories.append(LabelsWithCategory.End(builder))

    return label_categories


def createLabelWithInstanceWithCategory(
    builder: Builder,
    category: str,
    labels: List[str],
    instance_uuids=List[str],
    confidences: Union[List[float], None] = None,
) -> int:
    """
    Adds generates a list of labels with instances and adds it to a category.

    Args:
        builder: A flatbuffers Builder
        category: A category
        labels: A list of labels
        instance_uuids: A list of instance UUIDs
        confidences: A list of confidence values corresponding to the labels on the same index

    Returns:
        A pointer to the constructed\
        [LabelWithCategoryWithInstance](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/labels_with_instance_with_category.fbs)\
        object
    """
    cat_string = builder.CreateString(category)
    list_labels_w_instance = createLabelsWithInstance(builder, labels, instance_uuids, confidences)

    LabelsWithInstanceWithCategory.StartLabelsWithInstanceVector(builder, len(list_labels_w_instance))
    for label_with_instance in reversed(list_labels_w_instance):
        builder.PrependUOffsetTRelative(label_with_instance)
    labels_offset = builder.EndVector()

    LabelsWithInstanceWithCategory.Start(builder)
    LabelsWithInstanceWithCategory.AddCategory(builder, cat_string)
    LabelsWithInstanceWithCategory.AddLabelsWithInstance(builder, labels_offset)

    return LabelsWithInstanceWithCategory.End(builder)


def createLabelsWithInstanceWithCategory(
    builder: Builder,
    categories: List[str],
    cat_labels: List[List[str]],
    cat_instance_uuids: List[List[str]],
    cat_confidences: Union[List[List[float]], None] = None,
) -> List[int]:
    """
    Adds the same list of labels to each category of a list of categories.

    Args:
        builder: A flatbuffers Builder
        categories: A list with categories
        cat_labels: A inner list of labels for each category the outer list corresponding to the categories
        cat_instance_uuids: A inner list of instance UUIDs for each category with the outer\
            list corresponding to the categories
        cat_confidences: A inner list of confidence values for each category with the outer\
            list corresponding to the categories

    Returns:
        A list containing pointers to\
        [LabelWithCategoryWithInstance](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/labels_with_instance_with_category.fbs)\
        objects
    """

    assert len(categories) == len(cat_labels) == len(cat_instance_uuids)

    labels = []

    for quad in (
        zip(categories, cat_labels, cat_instance_uuids, cat_confidences)
        if cat_confidences
        else zip(categories, cat_labels, cat_instance_uuids)
    ):
        if len(quad) > 3:
            labels_w_instance_categories = createLabelWithInstanceWithCategory(
                builder, quad[0], quad[1], quad[2], quad[3]
            )
        else:
            labels_w_instance_categories = createLabelWithInstanceWithCategory(builder, quad[0], quad[1], quad[2])

        labels.append(labels_w_instance_categories)

    return labels


def createLabelsWithInstance(
    builder: Builder,
    labels: List[str],
    instanceUuids: List[str],
    confidences: Union[List[float], None] = None,
) -> List[int]:
    """
    Creates multiple general labels mapped to instances.

    Args:
        builder: A flatbuffers Builder
        labels: A list of label strings
        confidences: A list of confidence values corresponding to the label on the same index
        instanceUuids: A list of instance UUIDs corresponding to the label on the same index

    Returns:
        A list of pointers to the constructed\
        [LabelWithInstance](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/label_with_instance.fbs)\
        objects
    """
    assert len(labels) == len(instanceUuids)
    labelsGeneral = []
    if confidences:
        for label, confidence, uuid in zip(labels, confidences, instanceUuids):
            labelsGeneral.append(createLabelWithInstance(builder, label, uuid, confidence))
    else:
        for label, uuid in zip(labels, confidences, instanceUuids):
            labelsGeneral.append(createLabelWithInstance(builder, label, uuid))
    return labelsGeneral


def createPoint2d(builder: Builder, x: float, y: float) -> int:
    """
    Creates a 2D point in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        x: The x-coordinate of the 2D point
        y: The y-coordinate of the 2D point

    Returns:
        A pointer to the constructed point object
    """
    Point2D.Start(builder)
    Point2D.AddX(builder, x)
    Point2D.AddY(builder, y)
    return Point.End(builder)


def createPoint(builder: Builder, x: float, y: float, z: float) -> int:
    """
    Creates a 3D point in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        x: The x-coordinate of the 3D point
        y: The y-coordinate of the 3D point
        z: The z-coordinate of the 3D point

    Returns:
        A pointer to the constructed point object
    """
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    Point.AddZ(builder, z)
    return Point.End(builder)


def createPointStamped(builder: Builder, point: int, header: int, labelGeneralCategoryVector: int) -> int:
    """
    Creates a 3D point stamped in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        point: The pointer to a 3D point object
        header: The pointer to the header object of the point
        labelGeneralCategoryVector: The pointer to a vector of\
            [LabelsWithInstanceWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/labels_with_category.fbs)\
            objects of the point

    Returns:
        A pointer to the constructed point object
    """

    PointStamped.Start(builder)
    PointStamped.AddPoint(builder, point)
    PointStamped.AddHeader(builder, header)
    PointStamped.AddLabelsGeneral(builder, labelGeneralCategoryVector)
    return PointStamped.End(builder)


def createBoundingBox(builder: Builder, centerPoint: int, spatialExtent: int, rotation: Union[int, None] = None) -> int:
    """
    Creates a 3D bounding box in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        centerPoint: The pointer to the 3D Point object of the center point of the bounding box
        spatialExtent: The pointer to the 3D Point object representing the spatial extent in x, y and z direction
        rotation: The rotation of the bounding box as a flatbuffers Quaternion

    Returns:
        A pointer to the constructed\
        [BoundingBox](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingbox.fbs) object
    """
    Boundingbox.Start(builder)
    Boundingbox.AddCenterPoint(builder, centerPoint)
    Boundingbox.AddSpatialExtent(builder, spatialExtent)
    if rotation:
        Boundingbox.AddRotation(builder, rotation)
    return Boundingbox.End(builder)


def createPolygon2D(builder: Builder, height: float, z: float, vertices: List[int]) -> int:
    """
    Create a 2D Polygon in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        height: The height of the polygon
        z: The z-coordinate of the polygon from which the height is measured
        vertices: A list of pointers to\
            [Point2D](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point2d.fbs)\
            objects as the vertices of the polygon

    Returns:
        A pointer to the constructed\
        [Polygon2D](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/polygon2d.fbs)
    """

    Polygon2D.StartVerticesVector(builder, len(vertices))
    for v in vertices:
        builder.PrependUOffsetTRelative(v)
    vertices_fb = builder.EndVector()

    Polygon2D.Start(builder)
    Polygon2D.AddHeight(builder, height)
    Polygon2D.AddZ(builder, z)
    Polygon2D.AddVertices(builder, vertices_fb)

    return Polygon2D.End(builder)


def addToBoundingBoxLabeledVector(builder: Builder, boundingBoxLabeledList: List[int]) -> int:
    """
    Adds list of boudingBoxLabeled into the labelsBbVector of a flatbuffers pointcloud2.

    Args:
        builder: A flatbuffers Builder
        boundingBoxLabeledList: A list of pointers to\
            [BoundingBoxLabeledWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingbox_labeled_with_category.fbs)

    Returns:
        A pointer to the constructed vector of bounding box labeled with category objects
    """
    PointCloud2.StartLabelsBbVector(builder, len(boundingBoxLabeledList))
    for bb in reversed(boundingBoxLabeledList):
        builder.PrependUOffsetTRelative(bb)
    return builder.EndVector()


def addToPointFieldVector(builder: Builder, pointFieldList: List[int]) -> int:
    """
    Adds a list of pointFields into the fieldsVector of a flatbuffers pointcloud2.

    Args:
        builder: A flatbuffers Builder
        pointFieldList: A list of pointers to\
            [PointField](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_field.fbs) objects

    Returns:
        A pointer to the constructed vector of point fields objects
    """
    PointCloud2.StartFieldsVector(builder, len(pointFieldList))
    for pointField in reversed(pointFieldList):
        builder.PrependUOffsetTRelative(pointField)
    return builder.EndVector()


def createQuery(
    builder: Builder,
    timeInterval: Union[int, None] = None,
    labels: Union[List[int], None] = None,
    mustHaveAllLabels: bool = False,
    projectUuids: List[str] = None,
    instanceUuids: List[str] = None,
    dataUuids: List[str] = None,
    withoutData: bool = False,
    polygon2d: Union[int, None] = None,
    fullyEncapsulated: bool = False,
    inMapFrame: bool = True,
    sortByTime: bool = False,
) -> int:
    """
    Create a query, all parameters are optional.

    Args:
        builder: A flatbuffers Builder
        timeInterval: The pointer to a TimeInterval object representing the time frame of the returned instances
        labels: A list of pointers to\
            [LabelsWithInstanceWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/labels_with_instance_with_category.fbs)\
            flatbuffers objects, which the instances should atleast have one of
        mustHaveAllLabels: A boolean indicating if the returned instances should have all the labels each
        projectUuids: A list of project UUIDs to execute the query on
        instanceUuids: A list of specific instance UUIDs to query
        dataUuids: A list of specific data UUIDs to query
        withoutData: A boolean indicating if the query should return instances without their attached data
        polygon2d: A pointer to a Polygon2D object to retrieve only instances within the polygon
        fullyEncapsulated: A boolean indicating if the returned instances should be fully encapsulated by the polygon
        inMapFrame: A boolean indicating if the polygon coordinates are in the map frame or in EPSG world coordinates
        sortByTime: A boolean indicating if the returned instances should be sorted by time

    Returns:
        A pointer to the constructed query object
    """

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

    if labels:
        Query.QueryStartLabelVector(builder, len(labels))
        for label in reversed(labels):
            builder.PrependUOffsetTRelative(label)
        labelOffset = builder.EndVector()

    Query.Start(builder)
    if polygon2d:
        Query.AddPolygon(builder, polygon2d)
    if timeInterval:
        Query.AddTimeinterval(builder, timeInterval)
    if labels:
        Query.AddLabel(builder, labelOffset)
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


def createQueryInstance(builder: Builder, query: int, datatype: int) -> int:
    """
    Create a query for instances.

    Args:
        builder: A flatbuffers Builder
        query: The pointer to the query object
        datatype: The [Datatype](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/datatype.fbs)\
            of the instances to query

    Returns:
        A pointer to the constructed query instance object
    """
    QueryInstance.Start(builder)
    QueryInstance.AddDatatype(builder, datatype)
    QueryInstance.AddQuery(builder, query)
    return QueryInstance.End(builder)


def createTimeInterval(builder: Builder, timeMin: int, timeMax: int) -> int:
    """
    Create a closed time interval in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        timeMin: The pointer to a Timestamp object representing the lower bound of the time of the interval
        timeMax: The pointer to a Timestamp object representing the upper bound of the time of the interval

    Returns:
        A pointer to the constructed time interval object
    """
    TimeInterval.Start(builder)
    TimeInterval.AddTimeMin(builder, timeMin)
    TimeInterval.AddTimeMax(builder, timeMax)
    return TimeInterval.End(builder)


def createTransformStampedQuery(builder: Builder, header: int, childFrameId: str) -> int:
    """
    Create a transform stamped query in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        header: The pointer to a header object including the parent frame id
        childFrameId: The child frame id

    Returns:
        A pointer to the constructed transform stamped query object
    """
    TransformStampedQuery.Start(builder)
    TransformStampedQuery.AddHeader(builder, header)
    TransformStampedQuery.AddChildFrameId(builder, childFrameId)
    return TransformStampedQuery.End(builder)


def createRegionOfInterest(
    builder: Builder, x_offset: int, y_offset: int, height: int, width: int, do_rectify: bool
) -> int:
    """
    Create a region of interest in flatbuffers. This has the same structure as the ROS message\
    [RegionOfInterest](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RegionOfInterest.html) type.

    Args:
        builder: A flatbuffers Builder
        x_offset: Leftmost pixel of the region
        y_offset: Topmost pixel of the region
        height: The height of the region
        width: The width of the region
        do_rectify: A boolean indicating if a distinct ROI should be calculated

    Returns:
        A pointer to the constructed region of interest object
    """
    RegionOfInterest.Start(builder)
    RegionOfInterest.AddXOffset(builder, x_offset)
    RegionOfInterest.AddYOffset(builder, y_offset)
    RegionOfInterest.AddHeight(builder, height)
    RegionOfInterest.AddWidth(builder, width)
    RegionOfInterest.AddDoRectify(builder, do_rectify)
    return RegionOfInterest.End(builder)


def createCameraIntrinsics(
    builder: Builder,
    header: int,
    height: int,
    width: int,
    distortion_model: str,
    distortion: List[float],
    intrinsics_matrix: List[float],
    rectification_matrix: List[float],
    projection_matrix: List[float],
    binning_x: int,
    binning_y: int,
    region_of_interest: int,
    max_viewing_dist: float,
) -> int:
    """
    Create a camera intrinsics object in flatbuffers.
    This is structured in the same way as the ROS message\
    [CameraInfo](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html).

    Args:
        builder: A flatbuffers Builder
        header: The pointer to the header object of the camera intrinsics
        height: The height of the camera image
        width: The width of the camera image
        distortion_model: The distortion model of the camera
        distortion: The distortion coefficients of the camera
        intrinsics_matrix: The intrinsics matrix of the camera
        rectification_matrix: The rectification matrix of the camera
        projection_matrix: The projection matrix of the camera
        binning_x: The binning in horizontal direction
        binning_y: The binning in vertical direction
        region_of_interest: The region of interest of the camera
        max_viewing_dist: The maximum viewing distance of the camera

    Returns:
        A pointer to the constructed camera intrinsics object.
    """
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


def createCameraIntrinsicsQuery(builder: Builder, ci_uuid: str, project_uuid: str) -> str:
    """
    Create a query for camera intrinsics objects in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        ci_uuid: The UUID of the camera intrinsics
        project_uuid: The UUID of the project to query the camera intrinsics from

    Returns:
        A pointer to the constructed camera intrinsics query object
    """
    ci_uuid_str = builder.CreateString(ci_uuid)
    project_uuid_str = builder.CreateString(project_uuid)
    CameraIntrinsicsQuery.Start(builder)
    CameraIntrinsicsQuery.AddUuidCameraIntrinsics(builder, ci_uuid_str)
    CameraIntrinsicsQuery.AddUuidProject(builder, project_uuid_str)

    return CameraIntrinsicsQuery.End(builder)


def createUuidDatatypePair(builder: Builder, uuid: str, datatype: int) -> int:
    """
    Create a UUID datatype pair in flatbuffers for meta type services.

    Args:
        builder: A flatbuffers Builder
        uuid: The UUID of a project to retrieve the information from
        datatype: The [Datatype](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/datatype.fbs)\
            of the instance to retrieve the information from

    Returns:
        A pointer to the constructed UUID datatype pair object
    """
    uuidStr = builder.CreateString(uuid)

    UuidDatatypePair.Start(builder)
    UuidDatatypePair.AddProjectuuid(builder, uuidStr)
    UuidDatatypePair.AddDatatype(builder, datatype)
    return UuidDatatypePair.End(builder)


def createUuidDatatypeWithCategory(builder: Builder, uuid: str, datatype: int, category: str) -> int:
    """
    Create a UUID datatype pair with a category in flatbuffers for meta type services.

    Args:
        builder: A flatbuffers Builder
        uuid: The UUID of a project to retrieve the information from
        datatype: The [Datatype](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/datatype.fbs)\
            of the instance to retrieve the information from
        category: The category to which the instance belongs

    Returns:
        A pointer to the constructed UUID datatype with category object
    """
    categoryStr = builder.CreateString(category)

    UuidDatatypePair = createUuidDatatypePair(builder, uuid, datatype)

    UuidDatatypeWithCategory.Start(builder)
    UuidDatatypeWithCategory.AddCategory(builder, categoryStr)
    UuidDatatypeWithCategory.AddUuidAndDatatype(builder, UuidDatatypePair)
    return UuidDatatypeWithCategory.End(builder)


def createProjectInfo(builder: Builder, name: str, uuid: str) -> int:
    """
    Create a project info object in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        name: The name of the project
        uuid: The UUID of the project

    Returns:
        A pointer to the constructed project info object
    """
    nameStr = builder.CreateString(name)
    uuidStr = builder.CreateString(uuid)

    ProjectInfo.Start(builder)
    ProjectInfo.AddName(builder, nameStr)
    ProjectInfo.AddUuid(builder, uuidStr)
    return ProjectInfo.End(builder)


def createVector3(builder: Builder, t: Tuple[float, float, float]) -> int:
    """
    Create a 3D vector in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        t: The 3D vector to be created

    Returns:
        A pointer to the constructed 3D vector object
    """
    Vector3.Start(builder)
    Vector3.AddX(builder, t[0])
    Vector3.AddY(builder, t[1])
    Vector3.AddZ(builder, t[2])
    return Vector3.End(builder)


def createQuaternion(builder: Builder, quat: Quaternion.Quaternion) -> int:
    """
    Create a quaternion in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        quat: The quaternion to be created

    Returns:
        A pointer to the constructed quaternion object
    """
    Quaternion.Start(builder)
    Quaternion.AddX(builder, quat.x)
    Quaternion.AddY(builder, quat.y)
    Quaternion.AddZ(builder, quat.z)
    Quaternion.AddW(builder, quat.w)
    return Quaternion.End(builder)


def createTransform(builder: Builder, t: int, quat: int) -> int:
    """
    Create a transform in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        t: A pointer to a Vector3 representing the translation of the transform
        quat: The pointer to the Quaternion flatbuffers object of the transform

    Returns:
        A pointer to the constructed transform object
    """
    Transform.Start(builder)
    Transform.AddTranslation(builder, t)
    Transform.AddRotation(builder, quat)
    return Transform.End(builder)


def createTransformStamped(builder: Builder, childFrame: str, headerTf: int, transform: int) -> int:
    """
    Create a stamped transform in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        childFrame: The child frame of the transform
        headerTf: The pointer to the flatbuffers header of the transform
        transform: The pointer to the flatbuffers transform

    Returns:
        A pointer to the constructed stamped transform object
    """
    childFrame = builder.CreateString(childFrame)
    TransformStamped.Start(builder)
    TransformStamped.AddChildFrameId(builder, childFrame)
    TransformStamped.AddHeader(builder, headerTf)
    TransformStamped.AddTransform(builder, transform)
    return TransformStamped.End(builder)


def createImage(
    builder: Builder,
    image: np.ndarray,
    header: int,
    encoding: str,
    boundingBox2dLabeledVector: Union[List[int], None] = None,
    labelsGeneral: Union[List[int], None] = None,
) -> int:
    """
    Create an image in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        image: The image to be created (can be read from a file using imageio.imread())
        header: The pointer to the header object of the image
        encoding: The encoding of the image, e.g. rgb8
        boundingBox2dLabeledVector: A list of pointers to\
            [BoundingBox2DLabeledWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingbox2d_labeled_with_category.fbs)\
            objects to attach to the image
        labelsGeneral: A list to pointers to objects of type\
            [LabelsWithInstanceWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/labels_with_instance_with_category.fbs)

    Returns:
        A pointer to the constructed image object
    """
    encoding = builder.CreateString(encoding)

    if boundingBox2dLabeledVector:
        Image.StartLabelsBbVector(builder, len(boundingBox2dLabeledVector))
        for bb in reversed(boundingBox2dLabeledVector):
            builder.PrependUOffsetTRelative(bb)
        bbs = builder.EndVector()

    if labelsGeneral:
        Image.StartLabelsGeneralVector(builder, len(labelsGeneral))
        for label in reversed(labelsGeneral):
            builder.PrependUOffsetTRelative(label)
        labelsGeneralVector = builder.EndVector()

    imData = builder.CreateByteVector(image.tobytes())
    Image.Start(builder)
    Image.AddHeader(builder, header)
    Image.AddHeight(builder, image.shape[0])
    Image.AddWidth(builder, image.shape[1])
    Image.AddEncoding(builder, encoding)
    Image.AddStep(builder, 3 * image.shape[1])
    Image.AddData(builder, imData)
    if boundingBox2dLabeledVector:
        Image.AddLabelsBb(builder, bbs)
    if labelsGeneral:
        Image.AddLabelsGeneral(builder, labelsGeneralVector)
    return Image.End(builder)
