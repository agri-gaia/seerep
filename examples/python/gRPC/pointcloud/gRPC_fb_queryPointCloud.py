import sys
from typing import List

import flatbuffers
import numpy as np
from grpc import Channel
from seerep.fb import PointCloud2
from seerep.fb import point_cloud_service_grpc_fb as pointCloudService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createLabelsWithCategories,
    createPoint2d,
    createPolygon2D,
    createQuery,
    createTimeInterval,
    createTimeStamp,
    getProject,
    rosToNumpyDtype,
)
from seerep.util.fb_to_dict import fb_obj_to_dict

PROJECTNAME = "testproject"
NUM_BB_LABELS = 1
NUM_GENERAL_LABELS = 10


# TODO: move to module in the seerep.fb library
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


def query_pcs(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[PointCloud2.PointCloud2]:
    fb_builder = flatbuffers.Builder(1024)

    pcl_stub = pointCloudService.PointCloudServiceStub(grpc_channel)

    if target_proj_uuid is None:
        target_proj_uuid = getProject(fb_builder, grpc_channel, PROJECTNAME)

        if target_proj_uuid is None:
            print(f"Project: {PROJECTNAME} does not exist")
            sys.exit()

    # ruff: noqa: F841
    # create a polygon for a spatial query
    polygon_vertices = [createPoint2d(fb_builder, x, y) for x in [-10, 10] for y in [-10, 10]]
    query_polygon = createPolygon2D(fb_builder, 7, -1, polygon_vertices)

    # create a time interval for a temporal query
    time_min, time_max = [createTimeStamp(fb_builder, time) for time in [1610549273, 1938549273]]
    time_interval = createTimeInterval(fb_builder, time_min, time_max)

    # create labels for a semantic query
    category = ["myCategory"]
    labels = {
        "myCategory": [(fb_builder.CreateString(f"GeneralLabel{i}"), i * 10.0) for i in range(NUM_GENERAL_LABELS)]
    }

    label_wcategories = createLabelsWithCategories(fb_builder, category, labels)

    # filter for specific data
    data_uuids = [fb_builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
    instance_uuids = [fb_builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

    project_uuids = [fb_builder.CreateString(target_proj_uuid)]

    # set the query parameters according to your needs
    query = createQuery(
        fb_builder,
        # timeInterval=time_interval,
        labels=label_wcategories,
        # mustHaveAllLabels=False,
        # projectUuids=project_uuids,
        # instanceUuids=instance_uuids,
        # dataUuids=data_uuids,
        # withoutData=True,
        # polygon2d=query_polygon,
        # fullyEncapsulated=True
        inMapFrame=False,
    )

    fb_builder.Finish(query)

    resp_list = [
        PointCloud2.PointCloud2.GetRootAs(resp) for resp in pcl_stub.GetPointCloud2(bytes(fb_builder.Output()))
    ]

    return resp_list


if __name__ == "__main__":
    query_pcl = query_pcs()
    if not len(query_pcl):
        print("no response received")
    for resp in query_pcl:
        print(fb_obj_to_dict(resp))
        print(f"---Header--- \n {unpack_header(resp)}")

        point_fields = unpack_point_fields(resp)
        print(f"---Point Fields--- \n {point_fields}")

        print("---Bounding Box Labels---")
        for i in range(resp.LabelsBbLength()):
            for j in range(resp.LabelsBb(i).BoundingBoxLabeledLength()):
                print(
                    f"Label Label_BbLabeled {i}_{j}: \
                    {resp.LabelsBb(i).BoundingBoxLabeled(j).LabelWithInstance().Label().Label().decode('utf-8')}"
                )
                print(
                    f"Instance Label_BbLabeled {i}_{j}: \
                    {resp.LabelsBb(i).BoundingBoxLabeled(j).LabelWithInstance().InstanceUuid().decode('utf-8')}"
                )
                print(
                    f"Bounding Box Spatial Extent Label_BbLabeled {i}_{j}: "
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().SpatialExtent().X()},"
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().SpatialExtent().Y()},"
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().SpatialExtent().Z()} "
                    f"(x,y,z)"
                )
                print(
                    f"Bounding Box Center Point Label_BbLabeled {i}_{j}: "
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().CenterPoint().X()},"
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().CenterPoint().Y()},"
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().CenterPoint().Z()} "
                    f"(x,y,z)"
                )
                print(
                    f"Bounding Box Rotation Label_BbLabeled {i}_{j}: "
                    f"({resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().Rotation().X()}, "
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().Rotation().Y()}, "
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().Rotation().Z()}, "
                    f"{resp.LabelsBb(i).BoundingBoxLabeled(j).BoundingBox().Rotation().W()}) "
                    f"(x,y,z,w)"
                )

        print("---General Labels----")
        for i in range(resp.LabelsGeneralLength()):
            for j in range(resp.LabelsGeneral(i).LabelsWithInstanceLength()):
                print(f"Label {i}, {j}: {resp.LabelsGeneral(i).LabelsWithInstance(j).Label().Label().decode('utf-8')}")
                print(
                    f"Instance {i}, {j}: {resp.LabelsGeneral(i).LabelsWithInstance(j).InstanceUuid().decode('utf-8')}"
                )

        if not resp.DataIsNone():
            dtypes = np.dtype(
                {
                    "names": point_fields["name"],
                    "formats": [rosToNumpyDtype(datatype) for datatype in point_fields["datatype"]],
                    "offsets": point_fields["offset"],
                    "itemsize": resp.PointStep(),
                }
            )

            decoded_payload = np.frombuffer(resp.DataAsNumpy(), dtype=dtypes)

            print(f"---Is dense--- \n {resp.IsDense()}")
            print(f"---Payload--- \n {decoded_payload}")
            points = np.array([decoded_payload["x"], decoded_payload["y"], decoded_payload["z"]]).T.astype(np.float64)
