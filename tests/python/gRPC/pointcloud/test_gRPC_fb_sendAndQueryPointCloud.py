# test file for
#   gRPC_fb_queryPointCloud.py
#   gRPC_fb_sendPointCloud.py
from gRPC.pointcloud import gRPC_fb_queryPointCloud as query_pc
from gRPC.pointcloud import gRPC_fb_sendPointCloud as send_pc
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


# todo: test query dependant on tf
def test_sendAndQueryPCs(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    # send pointclouds to the project
    sent_pcs = sorted(
        [
            fb_flatc_dict(pcl, SchemaFileNames.POINT_CLOUD_2)
            for pcl in send_pc.send_pointcloud_raw(proj_uuid, grpc_channel)
        ],
        key=lambda pc: pc["header"]["uuid_msgs"],
    )

    queried_pcs = sorted(
        [fb_flatc_dict(pcl, SchemaFileNames.POINT_CLOUD_2) for pcl in query_pc.query_pcs_raw(proj_uuid, grpc_channel)],
        key=lambda pc: pc["header"]["uuid_msgs"],
    )

    assert queried_pcs == sent_pcs
