#!/usr/bin/env python3
import sys
import time
import uuid
from typing import Dict, List, Tuple, Union

import flatbuffers
import numpy as np
from grpc import Channel
from quaternion import quaternion
from seerep.fb import TransformStampedIntervalQuery
from seerep.fb import tf_service_grpc_fb as tf_srv
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createHeader,
    createQuaternion,
    createTimeStamp,
    createTransform,
    createTransformStamped,
    createTransformStampedQueryInterval,
    createVector3,
    getOrCreateProject,
)
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


# where time_min(inclusive) and time_max(exclusive)
# are of form Tuple[SECONDS, NANOSECONDS]
# and span the time interval to delete all tfs in between
def delete_tfs_raw(
    time_min: Tuple[int, int],
    time_max: Tuple[int, int],
    frame_id: str = "map",
    child_frame_id: str = "camera",
    target_proj_uuid: Union[str, None] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> bytearray:
    builder = flatbuffers.Builder(1024)

    if target_proj_uuid is None:
        target_proj_uuid = getOrCreateProject(
            flatbuffers.Builder(), grpc_channel, "testproject"
        )

    stubTf = tf_srv.TfServiceStub(grpc_channel)

    timestamp_min = createTimeStamp(builder, time_min[0], time_min[1])
    timestamp_max = createTimeStamp(builder, time_max[0], time_max[1])

    cur_time_ns = time.time_ns()
    cur_time_s = int(cur_time_ns // 1e9)
    cur_timestamp = createTimeStamp(
        builder, cur_time_s, int(cur_time_ns - cur_time_s * 1e9)
    )

    # timestamp in header is irrelevant, but does need to be filled in
    header = createHeader(builder, cur_timestamp, frame_id, target_proj_uuid)

    tf_query_interval = createTransformStampedQueryInterval(
        builder, header, child_frame_id, timestamp_min, timestamp_max
    )
    builder.Finish(tf_query_interval)
    buf = builder.Output()

    bufList = [bytes(buf)]

    stubTf.DeleteTransformStamped(iter(bufList))
    return buf


def delete_tfs(
    time_min: Tuple[int, int],
    time_max: Tuple[int, int],
    frame_id: str = "map",
    child_frame_id: str = "camera",
    target_proj_uuid: Union[str, None] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> TransformStampedIntervalQuery.TransformStampedIntervalQuery:
    buf = delete_tfs_raw(
        time_min,
        time_max,
        frame_id,
        child_frame_id,
        target_proj_uuid,
        grpc_channel,
    )
    return (
        TransformStampedIntervalQuery.TransformStampedIntervalQuery.GetRootAs(
            buf
        )
    )


def send_artificial_tfs(
    grpc_channel: Channel,
    project_uuid: str,
    time_lst: List[Tuple[int, int]],
    frame_id: str,
    child_frame_id: str,
) -> List[Dict]:
    # first send tfs with artifical timestamps
    stub_tf = tf_srv.TfServiceStub(grpc_channel)
    builder = flatbuffers.Builder(1024)
    quat = createQuaternion(builder, quaternion(1, 0, 0, 0))

    tf_list = []
    sent_tfs_base: List[Dict] = []
    for idx, time_tuple in enumerate(time_lst):
        timestmp = createTimeStamp(builder, time_tuple[0], time_tuple[1])

        # when not giving a uuid the corresponding retrieved tf from server has
        # no uuid msg
        header = createHeader(
            builder, timestmp, frame_id, project_uuid, str(uuid.uuid4())
        )

        translation = createVector3(
            builder,
            np.array([300 * idx, 300 * idx, 30 * idx], dtype=np.float64),
        )

        tf = createTransform(builder, translation, quat)
        tf_s = createTransformStamped(builder, child_frame_id, header, tf)

        builder.Finish(tf_s)
        buf = builder.Output()
        sent_tfs_base.append(
            fb_flatc_dict(buf, SchemaFileNames.TRANSFORM_STAMPED)
        )
        tf_list.append(bytes(buf))

    stub_tf.TransferTransformStamped(iter(tf_list))
    return sent_tfs_base


if __name__ == "__main__":
    timestamp_nanos = 1245
    nanos_factor = 1e-9

    timestamps = [
        (t, timestamp_nanos) for t in range(1661336507, 1661336558, 10)
    ]

    grpc_channel = get_gRPC_channel()

    proj_uuid = getOrCreateProject(
        flatbuffers.Builder(), grpc_channel, "testproject"
    )

    if grpc_channel is None or proj_uuid is None:
        print("There is no project on the server or server is not reachable")
        sys.exit()

    # sent_tfs = send_artificial_tfs(
    #     grpc_channel, proj_uuid, timestamps[:-2], "map", "camera"
    # )
    del_tfs = delete_tfs(
        timestamps[2], timestamps[3], "map", "camera", proj_uuid, grpc_channel
    )
    print("deleted tfs")
