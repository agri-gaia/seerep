import sys
from typing import List

import flatbuffers
import numpy as np
from grpc import Channel
from seerep.fb import PointCloud2
from seerep.fb import point_cloud_service_grpc_fb as pointCloudService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    create_label,
    create_label_category,
    createPoint2d,
    createPolygon2D,
    createQuery,
    createTimeInterval,
    createTimeStamp,
    getProject,
    rosToNumpyDtype,
)

PROJECTNAME = "testproject"
NUM_LABELS = 10


# TODO: move to module in the seerep.fb library
def unpack_point_fields(point_cloud: PointCloud2.PointCloud2) -> dict:
    """Extract the point fields from a Flatbuffer pcl message"""
    return {
        "name": [
            point_cloud.Fields(i).Name().decode("utf-8")
            for i in range(point_cloud.FieldsLength())
        ],
        "datatype": [
            point_cloud.Fields(i).Datatype()
            for i in range(point_cloud.FieldsLength())
        ],
        "offset": [
            point_cloud.Fields(i).Offset()
            for i in range(point_cloud.FieldsLength())
        ],
        "count": [
            point_cloud.Fields(i).Count()
            for i in range(point_cloud.FieldsLength())
        ],
    }


def unpack_header(point_cloud: PointCloud2.PointCloud2) -> dict:
    """Extract the header from a Flatbuffer pcl message"""
    return {
        "uuid_msgs": point_cloud.Header().UuidMsgs().decode("utf-8"),
        "uuid_project": point_cloud.Header().UuidProject().decode("utf-8"),
        "frame_id": point_cloud.Header().FrameId().decode("utf-8"),
    }


def query_pcs_raw(
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
    polygon_vertices = [
        createPoint2d(fb_builder, x, y) for x in [-10, 10] for y in [-10, 10]
    ]
    query_polygon = createPolygon2D(fb_builder, 7, -1, polygon_vertices)

    # create a time interval for a temporal query
    time_min, time_max = [
        createTimeStamp(fb_builder, time) for time in [1610549273, 1938549273]
    ]
    time_interval = createTimeInterval(fb_builder, time_min, time_max)

    # create labels for a semantic query
    labelsStrings = [f"Label{i}" for i in range(10)]

    labels = []
    for i in range(len(labelsStrings)):
        labels.append(
            create_label(builder=fb_builder, label=labelsStrings[i], label_id=1)
        )
    labelsCategory = []
    labelsCategory.append(
        create_label_category(
            builder=fb_builder,
            labels=labels,
            datumaro_json="a very valid datumaro json",
            category="category Z",
        )
    )

    # filter for specific data
    data_uuids = ["3e12e18d-2d53-40bc-a8af-c5cca3c3b248"]
    instance_uuids = ["3e12e18d-2d53-40bc-a8af-c5cca3c3b248"]

    project_uuids = [target_proj_uuid]

    # set the query parameters according to your needs
    query = createQuery(
        fb_builder,
        # timeInterval=time_interval,
        labels=labelsCategory,
        # mustHaveAllLabels=False,
        # projectUuids=project_uuids,
        # instanceUuids=instance_uuids,
        # dataUuids=data_uuids,
        # withoutData=True,
        # polygon2d=query_polygon,
        # fullyEncapsulated=True
        # crsString="project",
    )

    fb_builder.Finish(query)

    return pcl_stub.GetPointCloud2(bytes(fb_builder.Output()))


def query_pcs(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[PointCloud2.PointCloud2]:
    return [
        PointCloud2.PointCloud2.GetRootAs(pcl_buf)
        for pcl_buf in query_pcs_raw(target_proj_uuid, grpc_channel)
    ]


if __name__ == "__main__":
    query_pcl = query_pcs()
    if not len(query_pcl):
        print("no response received")
    for resp in query_pcl:
        # print(fb_to_dict.fb_flatc_dict(resp, SchemaFileNames.POINT_CLOUD_2))
        print(f"---Header--- \n {unpack_header(resp)}")

        point_fields = unpack_point_fields(resp)
        print(f"---Point Fields--- \n {point_fields}")

        print("---General Labels----")
        for i in range(resp.LabelsLength()):
            for j in range(resp.Labels(i).LabelsLength()):
                print(
                    f"Label {i}, {j}: "
                    f"{resp.Labels(i).Labels(j).Label().decode('utf-8')}"
                )
                print(
                    f"Instance {i}, {j}: "
                    f"{resp.Labels(i).Labels(j).InstanceUuid().decode('utf-8')}"
                )

        if not resp.DataIsNone():
            dtypes = np.dtype(
                {
                    "names": point_fields["name"],
                    "formats": [
                        rosToNumpyDtype(datatype)
                        for datatype in point_fields["datatype"]
                    ],
                    "offsets": point_fields["offset"],
                    "itemsize": resp.PointStep(),
                }
            )

            decoded_payload = np.frombuffer(resp.DataAsNumpy(), dtype=dtypes)

            print(f"---Is dense--- \n {resp.IsDense()}")
            # print(f"---Payload--- \n {decoded_payload}")
            # points = np.array([decoded_payload["x"], decoded_payload["y"],
            # decoded_payload["z"]]).T.astype(np.float64)
