import flatbuffers
import numpy as np
import open3d as o3d
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

PROJECTNAME = "testproject"

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
    vis.create_window()

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


fb_builder = flatbuffers.Builder(1024)

channel = get_gRPC_channel()
pcl_stub = pointCloudService.PointCloudServiceStub(channel)

if not (projectuuid := getProject(fb_builder, channel, PROJECTNAME)):
    print(f"Project: {PROJECTNAME} does not exist, quitting ...")
    raise SystemExit

# create a polygon for a spatial query
polygon_vertices = [createPoint2d(fb_builder, x, y) for x in [-10, 10] for y in [-10, 10]]
query_polygon = createPolygon2D(fb_builder, 7, -1, polygon_vertices)

# create a time interval for a temporal query
time_min, time_max = [createTimeStamp(fb_builder, time) for time in [1610549273, 1938549273]]
time_interval = createTimeInterval(fb_builder, time_min, time_max)

# create labels for a semantic query
labels = [createLabelWithConfidence(fb_builder, f"testlabel_{i}", 1.0) for i in range(3)]
labels_with_category = createLabelWithCategory(fb_builder, ["default_category"], [labels])

# filter for specific data
data_uuids = [fb_builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
instance_uuids = [fb_builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

project_uuids = [fb_builder.CreateString(projectuuid)]

# set the query parameters according to your needs
query = createQuery(
    fb_builder,
    # timeInterval=time_interval,
    # labels=labels_with_category,
    # mustHaveAllLabels=True,
    projectUuids=project_uuids,
    # instanceUuids=instance_uuids,
    # dataUuids=data_uuids,
    withoutData=False,
    # polygon2d=query_polygon,
    # fullyEncapsulated=True
)

fb_builder.Finish(query)

for responseBuf in pcl_stub.GetPointCloud2(bytes(fb_builder.Output())):
    if response := PointCloud2.PointCloud2.GetRootAs(responseBuf):
        print(f"---Header--- \n {unpack_header(response)}")

        point_fields = unpack_point_fields(response)
        print(f"---Point Fields--- \n {point_fields}")

        # TODO: add printing of general and bounding box labels

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

            print(f"---Is dense--- \n {response.IsDense()}")
            print(f"---Payload--- \n {decoded_payload}")
            points = np.array([decoded_payload["x"], decoded_payload["y"], decoded_payload["z"]]).T.astype(np.float64)

            print("-" * 13)
            # Not using the keyboard lib because it requires root privileges in linux to access the raw device files.
            while True:
                print("Visualize the point cloud? (Y/N)", end=" ")
                user_input = input().lower()
                if user_input == "y" or not user_input:
                    draw_pcl(points)
                    break
                else:
                    break
    else:
        print("No response received")
