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
    rosToNumpyDtype,
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
    withoutData=False,
    # polygon2d=polygon2d,
    # fullyEncapsulated=True,
)
builder.Finish(query)
buf = builder.Output()


def unpack_point_fields(point_cloud: PointCloud2.PointCloud2) -> dict:
    """Extract the point fields from a Flatbuffer pcl message"""
    return {
        "name": [point_cloud.Fields(i).Name().decode("utf-8") for i in range(point_cloud.FieldsLength())],
        "datatype": [point_cloud.Fields(i).Datatype() for i in range(point_cloud.FieldsLength())],
        "offset": [point_cloud.Fields(i).Offset() for i in range(point_cloud.FieldsLength())],
        "count": [point_cloud.Fields(i).Count() for i in range(point_cloud.FieldsLength())],
    }


def unpack_header(point_cloud: PointCloud2.PointCloud2) -> dict:
    """Extract the header from a Flatbuffer pcl message"""
    return {
        "uuid_msgs": point_cloud.Header().UuidMsgs().decode("utf-8"),
        "uuid_project": point_cloud.Header().UuidProject().decode("utf-8"),
        "frame_id": point_cloud.Header().FrameId().decode("utf-8"),
    }


for responseBuf in stubPointCloud.GetPointCloud2(bytes(buf)):
    response = PointCloud2.PointCloud2.GetRootAs(responseBuf)

    print(f"---Header--- \n {unpack_header(response)}")

    point_fields = unpack_point_fields(response)
    print(f"---Point Fields--- \n {point_fields}")

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

    if not response.DataIsNone():
        dtypes = np.dtype(
            {
                "names": point_fields["name"],
                "formats": [rosToNumpyDtype(datatype) for datatype in point_fields["datatype"]],
                "offsets": point_fields["offset"],
                "itemsize": response.PointStep(),
            }
        )

        decoded_payload = np.frombuffer(response.DataAsNumpy(), dtype=dtypes)
        print(f"---Data--- \n {decoded_payload}")
