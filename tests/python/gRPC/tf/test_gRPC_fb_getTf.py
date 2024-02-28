# test file for
#   gRPC_fb_getTf.py
import uuid
from copy import deepcopy
from typing import Dict, List, Tuple

import flatbuffers
import numpy as np
from gRPC.tf import gRPC_fb_getTf as get_tfs
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
TIMESTAMPS = [(t, TIMESTAMP_NANOS) for t in range(1661336507, 1661336528, 10)]
NANOS_FACTOR = 1e-9

# test sending and querying tfs
def test_gRPC_fb_getTf(grpc_channel, project_setup):
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
        header = createHeader(builder, timestmp, frame_id, project_uuid, str(uuid.uuid4()))
        time_lst.append(time)

        translation = createVector3(builder, np.array([300 * idx, 300 * idx, 30 * idx], dtype=np.float64))

        tf = createTransform(builder, translation, quat)
        tf_s = createTransformStamped(builder, child_frame_id, header, tf)

        builder.Finish(tf_s)
        buf = builder.Output()
        sent_tfs_base.append(fb_obj_to_dict(TransformStamped.TransformStamped.GetRootAs(buf)))
        tf_list.append(bytes(buf))

    stub_tf.TransferTransformStamped(iter(tf_list))

    # remove uuid msgs from sent tfs, because  they are not in the query either (possible bug?)
    for obj in sent_tfs_base:
        obj["Header"]["UuidMsgs"] = ""

    # access transform stamped objects, retrieve their times and check whether the get_Tf service provides those transforms
    # todo: modularize tf by time calculation

    # currently queried_tfs do not include the uuid for the message itself
    queried_tfs_base: List[Dict] = [
        fb_obj_to_dict(obj) for obj in get_tfs.get_tfs(time_lst, project_uuid, grpc_channel)
    ]

    # check if the count of sent objects is the same as the retrieved
    assert len(sent_tfs_base) == len(queried_tfs_base)

    # check if the objects are the same
    assert sent_tfs_base == queried_tfs_base

    # check if interpolation of the tfs works correctly (set time = time[i] * interp_time_factor + (1-interp_time_factor) * time[i+1])
    interp_time_factor = 0.5
    interp_times: List[Tuple[int, int]] = []

    for idx in range(len(sent_tfs_base) - 1):
        # average the times by this factor
        interp_times.append(
            (
                int(sent_tfs_base[idx]["Header"]["Stamp"]["Seconds"] * interp_time_factor)
                + int((1 - interp_time_factor) * sent_tfs_base[idx + 1]["Header"]["Stamp"]["Seconds"]),
                int(sent_tfs_base[idx]["Header"]["Stamp"]["Nanos"] * interp_time_factor)
                + int((1 - interp_time_factor) * sent_tfs_base[idx + 1]["Header"]["Stamp"]["Nanos"]),
            )
        )

    # this should only affect the translations in this case, calculate the new translations using the factor
    # interp_tfs_secs = [elem[0] for elem in interp_times]

    interp_tfs = []
    for idx in range(len(sent_tfs_base) - 1):
        tf = deepcopy(sent_tfs_base[0])
        sent_idx_stamp = sent_tfs_base[idx]["Header"]["Stamp"]
        sent_idxpp_stamp = sent_tfs_base[idx + 1]["Header"]["Stamp"]
        # recalculate time factor, due to numeric errors in float and int calculations
        time_factor = (
            (interp_times[idx][0] - sent_idx_stamp["Seconds"])
            + (interp_times[idx][1] - sent_idx_stamp["Nanos"]) * NANOS_FACTOR
        ) / (
            sent_idxpp_stamp["Seconds"]
            - sent_idx_stamp["Seconds"]
            + (sent_idxpp_stamp["Nanos"] - sent_idx_stamp["Nanos"]) * NANOS_FACTOR
        )
        sent_idx_tf_translation = sent_tfs_base[idx]["Transform"]["Translation"]
        sent_idxpp_tf_translation = sent_tfs_base[idx + 1]["Transform"]["Translation"]
        tf["Transform"]["Translation"]["X"] = (
            sent_idx_tf_translation["X"] * (1 - time_factor) + time_factor * sent_idxpp_tf_translation["X"]
        )
        tf["Transform"]["Translation"]["Y"] = (
            sent_idx_tf_translation["Y"] * (1 - time_factor) + time_factor * sent_idxpp_tf_translation["Y"]
        )
        tf["Transform"]["Translation"]["Z"] = (
            sent_idx_tf_translation["Z"] * (1 - time_factor) + time_factor * sent_idxpp_tf_translation["Z"]
        )
        tf["Header"]["Stamp"]["Seconds"] = interp_times[idx][0]
        tf["Header"]["Stamp"]["Nanos"] = interp_times[idx][1]
        interp_tfs.append(tf)

    # get tf by the times
    queried_tfs_interp: List[Dict] = [
        fb_obj_to_dict(obj) for obj in get_tfs.get_tfs(interp_times, project_uuid, grpc_channel)
    ]

    assert interp_tfs == queried_tfs_interp
