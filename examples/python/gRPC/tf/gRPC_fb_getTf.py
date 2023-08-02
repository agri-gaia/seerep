#!/usr/bin/env python3

import flatbuffers
from seerep.fb import TransformStamped
from seerep.fb import tf_service_grpc_fb as tfService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createHeader,
    createTimeStamp,
    createTransformStampedQuery,
    getProject,
)

builder = flatbuffers.Builder(1024)
channel = get_gRPC_channel("local")
PROJECT_NAME = "testproject"
projectUuid = getProject(builder, channel, PROJECT_NAME)

if not projectUuid:
    print("project not found")
    exit()

stubTf = tfService.TfServiceStub(channel)
for time in range(1661336507, 1661336550, 10):  # range(1663003789, 1663003834, 10):
    builder = flatbuffers.Builder(1024)

    timeSec = time  # 1663003788#time# 1663003820#1661336516#1663003820 #
    timeNano = 4148245
    frame = "map"

    timestamp = createTimeStamp(builder, timeSec, timeNano)
    header = createHeader(builder, timestamp, frame, projectUuid)

    childFrameId = builder.CreateString("camera")

    tfQuery = createTransformStampedQuery(builder, header, childFrameId)
    builder.Finish(tfQuery)

    tfBuf = stubTf.GetTransformStamped(bytes(builder.Output()))

    if tfBuf:
        tf = TransformStamped.TransformStamped.GetRootAs(tfBuf)
    else:
        print("No tf received")
        continue

    print(f"\n\ntime: {time}")
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
