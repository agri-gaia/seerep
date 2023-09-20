# test file for
#  gRPC_fb_addCameraIntrinsics.py
#  gRPC_fb_getCameraIntrinsics.py

import fb_to_dict
from gRPC.images import gRPC_fb_addCameraIntrinsics as add_ci
from gRPC.images import gRPC_fb_getCameraIntrinsics as get_ci


def test_addAndGetCameraIntrinsics(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    sent_ci = add_ci.add_camintrins(
        target_proj_uuid=proj_uuid, grpc_channel=grpc_channel
    )

    sent_ci_dict = fb_to_dict.fb_obj_to_dict(sent_ci)

    queried_ci = get_ci.get_camintrins(
        sent_ci_dict["Header"]["UuidMsgs"], proj_uuid, grpc_channel
    )

    assert sent_ci_dict == fb_to_dict.fb_obj_to_dict(queried_ci)
