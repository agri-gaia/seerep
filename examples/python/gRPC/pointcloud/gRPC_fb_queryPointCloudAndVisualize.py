import sys
from typing import List

import flatbuffers
import numpy as np
import open3d as o3d
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


# TODO: move into visaualization module
def draw_pcl(points: np.ndarray, point_colors: np.ndarray = None, draw_origin=True) -> None:
    """Visualize a point cloud using Open3D

    Based on https://github.com/open-mmlab/OpenPCDet/blob/255db8f02a8bd07211d2c91f54602d63c4c93356/tools/visual_utils/open3d_vis_utils.py#L38
    """
    vis = o3d.visualization.Visualizer()
    # print("before stuck")
    vis.create_window()
    # print("after stuck")

    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.zeros(3)

    if draw_origin:
        vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=np.zeros(3)))

    pts = o3d.geometry.PointCloud()
    pts.points = o3d.utility.Vector3dVector(points)

    vis.add_geometry(pts)
    if point_colors:
        pts.colors = o3d.utility.Vector3dVector(point_colors)
    else:
        pts.colors = o3d.utility.Vector3dVector(np.ones((points.shape[0], 3)))

    vis.run()
    vis.destroy_window()


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
    labelsStrings = [f"Label{i}" for i in range(10)]

    labels = []
    for i in range(len(labelsStrings)):
        labels.append(create_label(builder=fb_builder, label=labelsStrings[i], label_id=1))
    labelsCategory = []
    labelsCategory.append(
        create_label_category(
            builder=fb_builder, labels=labels, datumaro_json="a very valid datumaro json", category="category Z"
        )
    )

    # filter for specific data
    data_uuids = [fb_builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
    instance_uuids = [fb_builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

    project_uuids = [fb_builder.CreateString(target_proj_uuid)]

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
        print(f"---Header--- \n {unpack_header(resp)}")

        point_fields = unpack_point_fields(resp)
        print(f"---Point Fields--- \n {point_fields}")

        print("---Labels----")
        for i in range(resp.LabelsLength()):
            for j in range(resp.Labels(i).LabelsLength()):
                print(f"Label {i}, {j}: {resp.Labels(i).Labels(j).Label().decode('utf-8')}")
                print(f"Instance {i}, {j}: {resp.Labels(i).Labels(j).InstanceUuid().decode('utf-8')}")

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

            print("-" * 13)
            # Not using the keyboard lib because it requires root privileges in linux to access the raw device files.
            while True:
                print("Visualize the point cloud? (Y/N)", end=" ")
                user_input = input().lower()
                if user_input and user_input == "y":
                    draw_pcl(points)
                    break
                else:
                    break
