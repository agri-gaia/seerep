# test file for
#   gRPC_fb_addCameraIntrinsics.py
#   gRPC_fb_getCameraIntrinsics.py

from gRPC.images import gRPC_fb_addCameraIntrinsics as add_ci
from gRPC.images import gRPC_fb_getCameraIntrinsics as get_ci
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def test_addAndGetCameraIntrinsics(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    sent_ci = add_ci.add_camintrins_raw(target_proj_uuid=proj_uuid, grpc_channel=grpc_channel)

    sent_ci_dict = fb_flatc_dict(sent_ci, SchemaFileNames.CAMERA_INTRINSICS)

    queried_ci = get_ci.get_camintrins_raw(sent_ci_dict["header"]["uuid_msgs"], proj_uuid, grpc_channel)

    assert sent_ci_dict == fb_flatc_dict(queried_ci, SchemaFileNames.CAMERA_INTRINSICS)
