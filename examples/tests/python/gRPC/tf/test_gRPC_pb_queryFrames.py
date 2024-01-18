# test file for
#   gRPC_pb_queryFrames.py
import uuid
from typing import Dict, Final, List, Tuple

import flatbuffers
import numpy as np
import yaml
from gRPC.tf import gRPC_pb_queryFrames as query_frames
from quaternion import quaternion
from seerep.fb import TransformStamped
from seerep.fb import tf_service_grpc_fb as tf_serv
from seerep.util.fb_helper import (
    createHeader,
    createQuaternion,
    createTimeStamp,
    createTransform,
    createTransformStamped,
    createVector3,
)
from seerep.util.fb_to_dict import fb_obj_to_dict

TIMESTAMP_NANOS = 1245
TIMESTAMPS = [(t, TIMESTAMP_NANOS) for t in range(1661336507, 1661336538, 10)]
NANOS_FACTOR = 1e-9

TF_BROADCASTER: Final[str] = "fromHDF5"


def test_gRPC_pb_queryFrames(project_setup, grpc_channel):
    _, project_uuid = project_setup

    # first send tfs with artifical timestamps
    stub_tf = tf_serv.TfServiceStub(grpc_channel)
    builder = flatbuffers.Builder(1024)
    frame_id = "map"
    child_frame_id = "camera"
    quat = createQuaternion(builder, quaternion(1, 0, 0, 0))

    tf_list = []
    sent_tfs_base: List[Dict] = []
    time_lst: List[Tuple[int, int]] = []

    for idx, time in enumerate(TIMESTAMPS):

        timestmp = createTimeStamp(builder, time[0], time[1])

        # when not giving a uuid the corresponding retrieved tf from server has no uuid msg
        header = createHeader(
            builder, timestmp, frame_id, project_uuid, str(uuid.uuid4())
        )
        time_lst.append(time)

        translation = createVector3(
            builder, np.array([300 * idx, 300 * idx, 300 * idx], dtype=np.float64)
        )

        tf = createTransform(builder, translation, quat)
        tf_s = createTransformStamped(builder, child_frame_id, header, tf)

        builder.Finish(tf_s)
        buf = builder.Output()
        sent_tfs_base.append(
            fb_obj_to_dict(TransformStamped.TransformStamped.GetRootAs(buf))
        )
        tf_list.append(bytes(buf))

    stub_tf.TransferTransformStamped(iter(tf_list))

    frames = query_frames.get_frames(project_uuid, grpc_channel)

    for frame in frames.frames:
        frame_dict = yaml.safe_load(frame)
        assert child_frame_id in frame_dict
        assert frame_dict[child_frame_id]["parent"] == frame_id
        max_time_tf = max(
            sent_tfs_base, key=lambda obj: obj["Header"]["Stamp"]["Seconds"]
        )["Header"]["Stamp"]["Seconds"]
        min_time_tf = min(
            sent_tfs_base, key=lambda obj: obj["Header"]["Stamp"]["Seconds"]
        )["Header"]["Stamp"]["Seconds"]
        assert frame_dict[child_frame_id]["most_recent_transform"] == float(max_time_tf)
        assert frame_dict[child_frame_id]["oldest_transform"] == float(min_time_tf)
        assert frame_dict[child_frame_id]["buffer_length"] == float(
            max_time_tf - min_time_tf
        )
        assert frame_dict[child_frame_id]["broadcaster"] == TF_BROADCASTER
