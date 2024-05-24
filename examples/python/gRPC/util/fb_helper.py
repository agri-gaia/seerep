import sys
from typing import Dict, List, Tuple, Union

import numpy as np
from flatbuffers import Builder
from grpc import Channel
from seerep.fb import (
    Boundingbox,
    BoundingBox2DLabeled,
    BoundingBox2DLabeledWithCategory,
    BoundingBoxes2DLabeledStamped,
    BoundingBoxesLabeledStamped,
    BoundingBoxLabeled,
    BoundingBoxLabeledWithCategory,
    BoundingboxStamped,
    CameraIntrinsics,
    CameraIntrinsicsQuery,
    Datatype,
    Empty,
    GeodeticCoordinates,
    Header,
    Image,
    Label,
    LabelsWithCategory,
    LabelWithInstance,
    Point,
    Point_Field_Datatype,
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
            return {
                "name": response.Projects(i).Name().decode("utf-8"),
                "uuid": response.Projects(i).Uuid().decode("utf-8"),
                "frameid": response.Projects(i).Frameid().decode("utf-8"),
                "geodetic_position": response.Projects(i).GeodeticPosition(),
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
        coordSys: The coordinate system type as a EPSG code
        altitude: The altitude of the projects position on the globe (according to EPSG)
        latitude: The latitude of the project position on the globe (according to EPSG)
        longitude: The longitude of the project position on the globe (according to EPSG)

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
        coordSys: The coordinate system type as a EPSG code
        altitude: The altitude of the projects position on the globe (according to EPSG)
        latitude: The latitude of the projects position on the globe (according to EPSG)
        longitude: The longitude of the project positition on the globe (according to EPSG)

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
        coordSys: The coordinate system type as a EPSG code
        altitude: The altitude of the projects position on the globe (according to EPSG)
        latitude: The latitude of the project position on the globe (according to EPSG)
        longitude: The longitude of the project position on the globe (according to EPSG)
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


def createEmpty(builder: Builder) -> bytearray:
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
    return builder.Output()


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
        A pointer to the constructed timestamp object in memory.
    """
    Timestamp.Start(builder)
    Timestamp.AddSeconds(builder, seconds)
    Timestamp.AddNanos(builder, nanoseconds)
    return Timestamp.End(builder)


def createHeader(
    builder: Builder, timeStamp: int = None, frame: str = None, projectUuid: str = None, msgUuid: str = None
) -> int:
    """
    Creates a message header in flatbuffers, all parameters are optional.

    Args:
        builder: A flatbuffers Builder
        timeStamp: The pointer to a constructed Timestamp object in memory
        frame: The coordinate frame of the message
        projectUuid: The UUID of the project the message belongs to (this is often set by the server itself)
        msgUuid: The UUID of the message (when not provided it will be generated)

    Returns:
        A pointer to the constructed header object in memory.
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
def createPointField(
    builder: Builder, name: str, offset: int, datatype: Point_Field_Datatype.Point_Field_Datatype, count: int
) -> int:
    """
    Creates a point field to describe the structure of a point entry in the Pointcloud2 message.
    The same approach is used in the ROS \
    [Pointfield](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointField.html) message.

    Args:
        builder: A flatbuffers Builder
        name: The name of the point field
        offset: Offset from start of the point struct
        datatype: Datatype used for the point entries \
            (see [fbs definition](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_field.fbs))
        count: Number of elements in the point field

    Returns:
        A pointer to the constructed point field object in memory.
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
    datatype: Point_Field_Datatype.Point_Field_Datatype,
    dataTypeOffset: int,
    count: int,
) -> List[int]:
    """
    Creates point fields for all specified channels.

    Args:
        builder: A flatbuffers Builder
        channels: List of channel names
        datatype: Datatype used for the point entries \
            (see [fbs definition](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_field.fbs))
        dataTypeOffset: Offset from start of the point struct, in this case it is used for every channel
        count: Number of elements in each point field

    Returns:
        A list of pointers to the constructed point field objects in memory.
    """
    pointFieldsList = []
    offset = 0
    for channel in channels:
        pointFieldsList.append(createPointField(builder, channel, offset, datatype, count))
        offset += dataTypeOffset
    return pointFieldsList


def createLabelWithConfidence(builder: Builder, label: str, confidence: Union[float, None] = None) -> int:
    """
    Creates a label with an associated confidence value.

    Args:
        builder: A flatbuffers Builder
        label: The label string
        confidence: The confidence value of the label

    Returns:
        A pointer to the constructed label object in memory.
    """

    labelStr = builder.CreateString(label)
    Label.Start(builder)
    Label.AddLabel(builder, labelStr)
    if confidence:
        Label.AddConfidence(builder, confidence)
    return Label.End(builder)


def createLabelWithInstance(builder: str, label: str, confidence: Union[float, None], instanceUuid: str) -> int:
    """
    Creates a label with an associated instance uuid in flatbuffers

    Args:
        builder: A flatbuffers Builder
        label: The label string
        confidence: The confidence value of the label
        instanceUuid: The UUID of the instance which the label belongs to

    Returns:
        A pointer to the constructed label object in memory.
    """

    labelConfidence = createLabelWithConfidence(builder, label, confidence)
    instanceUuidStr = builder.CreateString(instanceUuid)
    LabelWithInstance.Start(builder)
    LabelWithInstance.AddLabel(builder, labelConfidence)
    LabelWithInstance.AddInstanceUuid(builder, instanceUuidStr)
    return LabelWithInstance.End(builder)


# category: list of categories
# labels: list of list of labels (as fb-String msg) per category
def createLabelWithCategory(builder: Builder, categories: List[str], labels: List[List[int]]) -> int:
    """
    Creates the message representing the labels of a category.

    Args:
        builder: A flatbuffers Builder
        categories: A list with categories
        labels: A list, which maps the categories to lists of labels according to the categories indices

    Returns:
        A pointer to the vector of category to labels mappings in memory.
    """

    LabelsCategories = []
    for iCategory in range(len(categories)):
        categoryStr = builder.CreateString(categories[iCategory])

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


def createLabelsWithCategories(
    builder: Builder, category: List[str], labels: List[str], confidences: List[float]
) -> List[int]:
    """
    Adds multiple of the same labels to a list of categories.

    Args:
        builder: A flatbuffers Builder
        category: A list with categories
        labels: A list of labels
        confidences: A list of confidence values corresponding to the label on the same index

    Returns:
        A pointer to the vector of category to labels mappings in memory.
    """
    label_categories = []

    for cat in category:
        cat_str = builder.CreateString(str(cat))
        labels_processed = [
            createLabelWithConfidence(builder, label, confidence) for label, confidence in zip(labels, confidences)[cat]
        ]

        LabelsWithCategory.StartLabelsVector(builder, len(labels_processed))
        for label in reversed(labels_processed):
            builder.PrependUOffsetTRelative(label)
        labels_offset = builder.EndVector()

        LabelsWithCategory.Start(builder)
        LabelsWithCategory.AddCategory(builder, cat_str)
        LabelsWithCategory.AddLabels(builder, labels_offset)
        label_categories.append(LabelsWithCategory.End(builder))

    return label_categories


def createLabelsWithInstance(
    builder: Builder, labels: List[str], confidences: List[float], instanceUuids: List[str]
) -> List[int]:
    """
    Creates multiple general labels mapped to instances.

    Args:
        builder: A flatbuffers Builder
        labels: A list of labels
        confidences: A list of confidence values corresponding to the label on the same index
        instanceUuids: A list of instance UUIDs corresponding to the label on the same index

    Returns:
        A list of pointers to the constructed label objects in memory.
    """
    assert len(labels) == len(instanceUuids)
    labelsGeneral = []
    for label, confidence, uuid in zip(labels, confidences, instanceUuids):
        labelsGeneral.append(createLabelWithInstance(builder, label, confidence, uuid))
    return labelsGeneral


def createPoint2d(builder: Builder, x: float, y: float) -> int:
    """
    Creates a 2D point in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        x: The x-coordinate of the 2D point
        y: The y-coordinate of the 2D point

    Returns:
        A pointer to the constructed point object in memory.
    """
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    return Point.End(builder)


def createBoundingBox2d(builder: Builder, centerPoint: int, spatialExtent: int, rotation: float = 0) -> int:
    """
    Creates a 2D bounding box in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        centerPoint: The pointer to the point object of the center point of the bounding box
        spatialExtent: The pointer to the point object representing the spatial extent in x and y direction
        rotation: The rotation of the bounding box

    Returns:
        A pointer to the constructed bounding box object in memory.
    """
    Boundingbox.Start(builder)
    Boundingbox.AddCenterPoint(builder, centerPoint)
    Boundingbox.AddSpatialExtent(builder, spatialExtent)
    Boundingbox.AddRotation(builder, rotation)
    return Boundingbox.End(builder)


def createBoundingBox2dLabeled(builder: Builder, instance: int, boundingBox: int) -> int:
    """
    Creates a 2d bounding box with label in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        instance: A pointer to the LabelWithInstance object of the bounding box
        boundingBox: The pointer to a \
            [Boundingbox2D](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingbox2d.fbs) object

    Returns:
        A pointer to the constructed bounding box object in memory.
    """

    BoundingBox2DLabeled.Start(builder)
    BoundingBox2DLabeled.AddLabelWithInstance(builder, instance)
    BoundingBox2DLabeled.AddBoundingBox(builder, boundingBox)
    return BoundingBox2DLabeled.End(builder)


def createBoundingBoxes2d(builder: Builder, centerPoints: List[int], spatialExtents: List[int]) -> List[int]:
    """
    Creates multiple 2D bounding boxes in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        centerPoints: A list of pointers to the point objects of the center points of the bounding boxes
        spatialExtents: A list of pointers to the point objects representing the spatial extents in x and y direction

    Returns:
        A list of pointers to the constructed \
            [Boundingbox2D](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingbox2d.fbs) \
            objects in memory.
    """
    assert len(centerPoints) == len(spatialExtents)
    boundingBoxes = []
    for centerPoint, spatialExtent in zip(centerPoints, spatialExtents):
        boundingBoxes.append(createBoundingBox2d(builder, centerPoint, spatialExtent))
    return boundingBoxes


def createBoundingBoxes2dLabeled(builder: Builder, instances: List[int], boundingBoxes: List[int]) -> List[int]:
    """
    Creates multiple labeled 2D bounding boxes.

    Args:
        builder: A flatbuffers Builder
        instances: A list of pointers to the LabelWithInstance objects of the bounding boxes
        boundingBoxes: A list of pointers to 2D bounding box objects

    Returns:
        A list of pointers to the constructed bounding box objects in memory.
    """
    assert len(instances) == len(boundingBoxes)
    boundingBoxes2dLabeled = []
    for instance, boundingBox in zip(instances, boundingBoxes):
        boundingBoxes2dLabeled.append(createBoundingBox2dLabeled(builder, instance, boundingBox))
    return boundingBoxes2dLabeled


# ruff: noqa: E501
def createBoundingBox2dLabeledStamped(builder: Builder, header: int, labelsBb: List[int]) -> int:
    """
    Creates a labeled bounding box 2d in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        header: The pointer to the header object of the bounding box
        labelsBb: A list of pointers to \
            [BoundingBoxes2DLabeledStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingbox2d_labeled_with_category.fbs) \
            objects
    """
    BoundingBoxes2DLabeledStamped.StartLabelsBbVector(builder, len(labelsBb))
    for labelBb in reversed(labelsBb):
        builder.PrependUOffsetTRelative(labelBb)
    labelsBbVector = builder.EndVector()

    BoundingBoxes2DLabeledStamped.Start(builder)
    BoundingBoxes2DLabeledStamped.AddHeader(builder, header)
    BoundingBoxes2DLabeledStamped.AddLabelsBb(builder, labelsBbVector)
    return BoundingBoxes2DLabeledStamped.End(builder)


# ruff: noqa: E501
def createBoundingBoxLabeledStamped(builder: Builder, header: int, labelsBb: List[int]) -> int:
    """
    Creates a labeled bounding box in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        header: The pointer to the header object of the bounding box
        labelsBb: A list of pointers to \
            [BoundingBoxLabeledWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingbox_labeled_with_category.fbs) \
            objects
    """
    BoundingBoxesLabeledStamped.StartLabelsBbVector(builder, len(labelsBb))
    for labelBb in reversed(labelsBb):
        builder.PrependUOffsetTRelative(labelBb)
    labelsBbVector = builder.EndVector()

    BoundingBoxesLabeledStamped.Start(builder)
    BoundingBoxesLabeledStamped.AddHeader(builder, header)
    BoundingBoxesLabeledStamped.AddLabelsBb(builder, labelsBbVector)
    return BoundingBoxesLabeledStamped.End(builder)


def createBoundingBox2DLabeledWithCategory(builder: Builder, category: str, bb2dLabeled: List[int]) -> int:
    """ """
    BoundingBox2DLabeledWithCategory.StartBoundingBox2dLabeledVector(builder, len(bb2dLabeled))
    for labelBb in reversed(bb2dLabeled):
        builder.PrependUOffsetTRelative(labelBb)
    labelsBbVector = builder.EndVector()

    BoundingBox2DLabeledWithCategory.Start(builder)
    BoundingBox2DLabeledWithCategory.AddCategory(builder, category)
    BoundingBox2DLabeledWithCategory.AddBoundingBox2dLabeled(builder, labelsBbVector)
    return BoundingBox2DLabeledWithCategory.End(builder)


def createPoint(builder: Builder, x: float, y: float, z: float) -> int:
    """Creates a 3D point in flatbuffers"""
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    Point.AddZ(builder, z)
    return Point.End(builder)


def createPointStamped(builder: Builder, point: int, header: int, labelGeneralCategoryVector: int) -> int:
    """Creates a 3D point stamped in flatbuffers"""

    PointStamped.Start(builder)
    PointStamped.AddPoint(builder, point)
    PointStamped.AddHeader(builder, header)
    PointStamped.AddLabelsGeneral(builder, labelGeneralCategoryVector)
    return PointStamped.End(builder)


def createBoundingBox(builder: Builder, centerPoint: int, spatialExtent: int, rotation: float = None) -> int:
    """Creates a 3D bounding box in flatbuffers"""
    Boundingbox.Start(builder)
    Boundingbox.AddCenterPoint(builder, centerPoint)
    Boundingbox.AddSpatialExtent(builder, spatialExtent)
    if rotation:
        Boundingbox.AddRotation(builder, rotation)
    return Boundingbox.End(builder)


def createPolygon2D(builder: Builder, height: float, z: float, vertices: List[int]) -> int:
    """Create a 2D Polygon in flatbuffers"""

    Polygon2D.StartVerticesVector(builder, len(vertices))
    for v in vertices:
        builder.PrependUOffsetTRelative(v)
    vertices_fb = builder.EndVector()

    Polygon2D.Start(builder)
    Polygon2D.AddHeight(builder, height)
    Polygon2D.AddZ(builder, z)
    Polygon2D.AddVertices(builder, vertices_fb)

    return Polygon2D.End(builder)


def createBoundingBoxStamped(
    builder: Builder, header: int, centerPoint: int, spatialExtent: int, rotation: float = None
) -> int:
    """Creates a stamped 3D bounding box in flatbuffers"""
    boundingBox = createBoundingBox(builder, centerPoint, spatialExtent, rotation)
    BoundingboxStamped.Start(builder)
    BoundingboxStamped.AddHeader(builder, header)
    BoundingboxStamped.AddBoundingbox(builder, boundingBox)
    return BoundingboxStamped.End(builder)


def createBoundingBoxes(builder: Builder, centerPoint: int, spatialExtent: int, rotation: float = None):
    assert len(centerPoint) == len(spatialExtent)
    boundingBoxes = []
    if rotation:
        for center, extent, rot in zip(centerPoint, spatialExtent, rotation):
            boundingBoxes.append(createBoundingBox(builder, center, extent, rot))
    else:
        for center, extent in zip(centerPoint, spatialExtent):
            boundingBoxes.append(createBoundingBox(builder, center, extent))
    return boundingBoxes


def createBoundingBoxLabeled(builder: Builder, instance: int, boundingBox: int):
    """Creates a labeled bounding box in flatbuffers"""
    BoundingBoxLabeled.Start(builder)
    BoundingBoxLabeled.AddLabelWithInstance(builder, instance)
    BoundingBoxLabeled.AddBoundingBox(builder, boundingBox)
    return BoundingBoxLabeled.End(builder)


def createBoundingBoxesLabeled(builder: Builder, instances: List[int], boundingBoxes: List[int]) -> List[int]:
    """Creates multiple labeled bounding boxes"""
    assert len(instances) == len(boundingBoxes)
    boundingBoxesLabeled = []
    for instance, boundingBox in zip(instances, boundingBoxes):
        boundingBoxesLabeled.append(createBoundingBoxLabeled(builder, instance, boundingBox))
    return boundingBoxesLabeled


def createBoundingBoxLabeledWithCategory(builder: Builder, category: str, bbLabeled: List[int]) -> int:
    BoundingBoxLabeledWithCategory.StartBoundingBoxLabeledVector(builder, len(bbLabeled))
    for labelBb in reversed(bbLabeled):
        builder.PrependUOffsetTRelative(labelBb)
    labelsBbVector = builder.EndVector()

    BoundingBoxLabeledWithCategory.Start(builder)
    BoundingBoxLabeledWithCategory.AddCategory(builder, category)
    BoundingBoxLabeledWithCategory.AddBoundingBoxLabeled(builder, labelsBbVector)
    return BoundingBoxLabeledWithCategory.End(builder)


def addToBoundingBoxLabeledVector(builder: Builder, boundingBoxLabeledList: List[int]) -> int:
    """Adds list of boudingBoxLabeled into the labelsBbVector of a flatbuffers pointcloud2"""
    PointCloud2.StartLabelsBbVector(builder, len(boundingBoxLabeledList))
    # Note: reverse because we prepend
    for bb in reversed(boundingBoxLabeledList):
        builder.PrependUOffsetTRelative(bb)
    return builder.EndVector()


def addToGeneralLabelsVector(builder: Builder, generalLabelList: List[int]) -> int:
    """Adds list of generalLabels into the labelsGeneralVector of a flatbuffers pointcloud2"""
    PointCloud2.StartLabelsGeneralVector(builder, len(generalLabelList))
    # Note: reverse because we prepend
    for label in reversed(generalLabelList):
        builder.PrependUOffsetTRelative(label)
    return builder.EndVector()


def addToPointFieldVector(builder: Builder, pointFieldList: List[int]) -> int:
    """Adds a list of pointFields into the fieldsVector of a flatbuffers pointcloud2"""
    PointCloud2.StartFieldsVector(builder, len(pointFieldList))
    # Note: reverse because we prepend
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
    polygon2d: Union[int] = None,
    fullyEncapsulated: bool = False,
    inMapFrame: bool = True,
    sortByTime: bool = False,
):
    """Create a query, all parameters are optional"""

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


def createQueryInstance(builder: Builder, query: int, datatype: Datatype.Datatype) -> int:
    """Create a query for instances"""
    QueryInstance.Start(builder)
    QueryInstance.AddDatatype(builder, datatype)
    QueryInstance.AddQuery(builder, query)
    return QueryInstance.End(builder)


def createTimeInterval(builder: Builder, timeMin: int, timeMax: int) -> int:
    """Create a time time interval in flatbuffers"""
    TimeInterval.Start(builder)
    TimeInterval.AddTimeMin(builder, timeMin)
    TimeInterval.AddTimeMax(builder, timeMax)
    return TimeInterval.End(builder)


def createTransformStampedQuery(builder: Builder, header: int, childFrameId: str) -> int:
    TransformStampedQuery.Start(builder)
    TransformStampedQuery.AddHeader(builder, header)
    TransformStampedQuery.AddChildFrameId(builder, childFrameId)
    return TransformStampedQuery.End(builder)


def createRegionOfInterest(builder: Builder, x_offset: int, y_offset: int, height: int, width: int, do_rectify: bool):
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
    ci_uuid_str = builder.CreateString(ci_uuid)
    project_uuid_str = builder.CreateString(project_uuid)
    CameraIntrinsicsQuery.Start(builder)
    CameraIntrinsicsQuery.AddUuidCameraIntrinsics(builder, ci_uuid_str)
    CameraIntrinsicsQuery.AddUuidProject(builder, project_uuid_str)

    return CameraIntrinsicsQuery.End(builder)


def createUuidDatatypePair(builder: Builder, uuid: str, datatype: Datatype.Datatype) -> int:
    uuidStr = builder.CreateString(uuid)

    UuidDatatypePair.Start(builder)
    UuidDatatypePair.AddProjectuuid(builder, uuidStr)
    UuidDatatypePair.AddDatatype(builder, datatype)
    return UuidDatatypePair.End(builder)


def createUuidDatatypeWithCategory(builder: Builder, uuid: str, datatype: Datatype.Datatype, category: str) -> int:
    categoryStr = builder.CreateString(category)

    UuidDatatypePair = createUuidDatatypePair(builder, uuid, datatype)

    UuidDatatypeWithCategory.Start(builder)
    UuidDatatypeWithCategory.AddCategory(builder, categoryStr)
    UuidDatatypeWithCategory.AddUuidAndDatatype(builder, UuidDatatypePair)
    return UuidDatatypeWithCategory.End(builder)


def createProjectInfo(builder: Builder, name: str, uuid: str) -> int:
    nameStr = builder.CreateString(name)
    uuidStr = builder.CreateString(uuid)

    ProjectInfo.Start(builder)
    ProjectInfo.AddName(builder, nameStr)
    ProjectInfo.AddUuid(builder, uuidStr)
    return ProjectInfo.End(builder)


def createVector3(builder: Builder, t: Tuple[float, float, float]) -> int:
    Vector3.Start(builder)
    Vector3.AddX(builder, t[0])
    Vector3.AddY(builder, t[1])
    Vector3.AddZ(builder, t[2])
    return Vector3.End(builder)


def createQuaternion(builder: Builder, quat: Quaternion.Quaternion) -> int:
    Quaternion.Start(builder)
    Quaternion.AddX(builder, quat.x)
    Quaternion.AddY(builder, quat.y)
    Quaternion.AddZ(builder, quat.z)
    Quaternion.AddW(builder, quat.w)
    return Quaternion.End(builder)


def createTransform(builder: Builder, t: Tuple[float, float, float], quat: Quaternion.Quaternion) -> int:
    Transform.Start(builder)
    Transform.AddTranslation(builder, t)
    Transform.AddRotation(builder, quat)
    return Transform.End(builder)


def createTransformStamped(builder: Builder, childFrame: str, headerTf: int, transform: int) -> int:
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
    boundingBox2dLabeledVector: Union[int, None] = None,
    labelsGeneral: Union[int, None] = None,
) -> int:
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
