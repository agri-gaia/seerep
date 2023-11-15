#!/usr/bin/env python3

from typing import List, Tuple

import flatbuffers
from google.protobuf import empty_pb2
from grpc import Channel
from seerep.fb import TransformStamped
from seerep.fb import tf_service_grpc_fb as tfService
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2 as projectCreation
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createHeader,
    createTimeStamp,
    createTransformStampedQuery,
)


# where tf_times_list is a list which contains elements of the form Tuple[SECONDS, NANOSECONDS]
def get_tfs(
    tf_times_list: List[Tuple[int, int]],
    target_proj_uuid: str = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> List[TransformStamped.TransformStamped]:
    builder = flatbuffers.Builder(1024)

    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)

    if target_proj_uuid is None:
        # 2. Get all projects from the server
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            print(project.name + " " + project.uuid)
            if project.name == "LabeledImagesInGrid":
                target_proj_uuid = project.uuid

        if target_proj_uuid is None:
            creation = projectCreation.ProjectCreation(
                name="LabeledImagesInGrid", mapFrameId="map"
            )
            projectCreated = stubMeta.CreateProject(creation)
            target_proj_uuid = projectCreated.uuid

    stubTf = tfService.TfServiceStub(grpc_channel)

    tf_per_time: List[TransformStamped.TransformStamped] = []

    for time in tf_times_list:
        time_sec = time[0]
        time_nano = time[1]

        frame = "map"

        timestamp = createTimeStamp(builder, time_sec, time_nano)
        header = createHeader(builder, timestamp, frame, target_proj_uuid)

        child_frame_id = builder.CreateString("camera")

        tf_query = createTransformStampedQuery(builder, header, child_frame_id)
        builder.Finish(tf_query)

        tf_buf = stubTf.GetTransformStamped(bytes(builder.Output()))

        if not tf_buf:
            print("No tf received")
            continue

        tf = TransformStamped.TransformStamped.GetRootAs(tf_buf)

        tf_per_time.append(tf)

    return tf_per_time


if __name__ == "__main__":
    tfs = get_tfs((time, 0) for time in range(1661336507, 1661336550, 10))
    for tf in tfs:
        print(
            f"\n\ntime: {tf.Header().Stamp().Sec()} Sec. {tf.Header().Stamp().NanoSec()} NanoSec"
        )
        print("parent frame: " + tf.Header().FrameId().decode("utf-8"))
        print("child frame: " + tf.ChildFrameId().decode("utf-8"))
        print(
            "translation: "
            + str(tf.Transform().Translation().X())
            + ";"
            + str(tf.Transform().Translation().Y())
            + ";"
            + str(tf.Transform().Translation().Z())
        )
        print(
            "quaternion: "
            + str(tf.Transform().Rotation().X())
            + ";"
            + str(tf.Transform().Rotation().Y())
            + ";"
            + str(tf.Transform().Rotation().Z())
            + ";"
            + str(tf.Transform().Rotation().W())
        )
