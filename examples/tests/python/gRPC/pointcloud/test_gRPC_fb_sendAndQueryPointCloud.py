from gRPC.pointcloud import gRPC_fb_queryPointCloud as query_pc
from gRPC.pointcloud import gRPC_fb_sendPointCloud as send_pc
from seerep.util import fb_to_dict


# todo: test query dependant on tf
def test_sendAndQueryPCs(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    # send pointclouds to the project
    sent_pcs = [
        fb_to_dict.fb_obj_to_dict(elem)
        for elem in sorted(
            send_pc.send_pointcloud(proj_uuid, grpc_channel),
            key=lambda pc: pc.Header().UuidMsgs().decode(),
        )
    ]
    queried_pcs = [
        fb_to_dict.fb_obj_to_dict(elem)
        for elem in sorted(
            query_pc.query_pcs(proj_uuid, grpc_channel),
            key=lambda pc: pc.Header().UuidMsgs().decode(),
        )
    ]

    assert queried_pcs == sent_pcs
