import struct
import sys

import flatbuffers
import numpy as np

np.set_printoptions(precision=7)

from seerep.fb import PointCloud2
from seerep.fb import point_cloud_service_grpc_fb as pointCloudService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createLabelWithCategory,
    createLabelWithConfidence,
    createPoint2d,
    createPolygon2D,
    createQuery,
    createTimeInterval,
    createTimeStamp,
    getProject,
)

channel = get_gRPC_channel()

stubPointCloud = pointCloudService.PointCloudServiceStub(channel)
builder = flatbuffers.Builder(1024)

PROJECTNAME = "testproject"
projectuuid = getProject(builder, channel, PROJECTNAME)

if projectuuid is None:
    print(f"Project: {PROJECTNAME} does not exist")
    sys.exit()


builder = flatbuffers.Builder(1024)

# Create all necessary objects for the query
l = 10
polygon_vertices = []
polygon_vertices.append(createPoint2d(builder, -1.0 * l, -1.0 * l))
polygon_vertices.append(createPoint2d(builder, -1.0 * l, l))
polygon_vertices.append(createPoint2d(builder, l, l))
polygon_vertices.append(createPoint2d(builder, l, -1.0 * l))
polygon2d = createPolygon2D(builder, 7, -1, polygon_vertices)

timeMin = createTimeStamp(builder, 1610549273, 0)
timeMax = createTimeStamp(builder, 1938549273, 0)
timeInterval = createTimeInterval(builder, timeMin, timeMax)

projectUuids = [builder.CreateString(projectuuid)]
# list of categories
category = ["0"]
# list of labels per category
labels = [
    [
        createLabelWithConfidence(builder, "testlabel0"),
        createLabelWithConfidence(builder, "testlabelgeneral0"),
    ]
]
labelCategory = createLabelWithCategory(builder, category, labels)
dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = createQuery(
    builder,
    # boundingBox=boundingboxStamped,
    # timeInterval=timeInterval,
    # labels=labelCategory,
    # mustHaveAllLabels=True,
    projectUuids=projectUuids,
    # instanceUuids=instanceUuids,
    # dataUuids=dataUuids,
    withoutData=True,
    polygon2d=polygon2d,
    fullyEncapsulated=True,
)
builder.Finish(query)
buf = builder.Output()

for responseBuf in stubPointCloud.GetPointCloud2(bytes(buf)):
    response = PointCloud2.PointCloud2.GetRootAs(responseBuf)

    print("---Header---")
    print(f"Message UUID: {response.Header().UuidMsgs().decode('utf-8')}")
    print(f"Project UUID: {response.Header().UuidProject().decode('utf-8')}")
    print(f"Frame ID: {response.Header().FrameId().decode('utf-8')}")

    print("---Point Fields---")
    for i in range(response.FieldsLength()):
        print(f"Field Name: {response.Fields(i).Name().decode('utf-8')}")
        print(f"Datatype: {response.Fields(i).Datatype()}")
        print(f"Offset: {response.Fields(i).Offset()}")
        print(f"Count: {response.Fields(i).Count()}")

    print("---Bounding Box Labels---")
    for i in range(response.LabelsBbLength()):
        print(f"Label {i}: {response.LabelsBb(i).LabelWithInstance().Label().decode('utf-8')}")
        print(f"Instance {i}: {response.LabelsBb(i).LabelWithInstance().InstanceUuid().decode('utf-8')}")
        print(
            f"Bounding Box Min {i}: "
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMin().X()},"
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMin().Y()},"
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMin().Z()} "
            f"(x,y,z)"
        )
        print(
            f"Bounding Box Max {i}: "
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMax().X()},"
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMax().Y()},"
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMax().Z()} "
            f"(x,y,z)"
        )

    print("---General Labels----")
    for i in range(response.LabelsGeneralLength()):
        print(f"Label {i}: {response.LabelsGeneral(i).Label().decode('utf-8')}")
        print(f"Instance {i}: {response.LabelsGeneral(i).InstanceUuid().decode('utf-8')}")

    print("---Data--")
    if not response.DataIsNone():
        rawData = response.DataAsNumpy()
        data = [struct.unpack('f', rawData[i : i + 4]) for i in range(0, rawData.shape[0], 4)]
        reshapedData = np.array(data).reshape(960, 1280, 4)
        print(f"Data: {reshapedData}")
