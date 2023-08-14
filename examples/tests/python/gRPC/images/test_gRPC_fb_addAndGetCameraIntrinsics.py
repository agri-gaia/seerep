from collections.abc import Iterable
from typing import Dict, List, Tuple

import conftest
import flatbuffers
from gRPC.images import gRPC_fb_addCameraIntrinsics as add_ci
from gRPC.images import gRPC_fb_getCameraIntrinsics as get_ci
from seerep.fb import CameraIntrinsics


def test_addAndGetCameraIntrinsics(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    sent_ci = add_ci.add_camintrins(
        target_proj_uuid=proj_uuid, grpc_channel=grpc_channel
    )

    conftest.fb_obj_to_dict(sent_ci)
