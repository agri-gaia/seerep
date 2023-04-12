#!/usr/bin/env python3

import os
import sys

import flatbuffers
from seerep.fb import TransformStamped, TransformStampedQuery
from seerep.fb import tf_service_grpc_fb as tfService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)
channel = util.get_gRPC_channel("local")
PROJECT_NAME = "plantmap01"
projectUuid = util_fb.getProject(builder, channel, PROJECT_NAME)

stubTf = tfService.TfServiceStub(channel)
for time in range(1661336507, 1661336550, 10):  # range(1663003789, 1663003834, 10):
    builder = flatbuffers.Builder(1024)

    timeSec = time  # 1663003788#time# 1663003820#1661336516#1663003820 #
    timeNano = 4148245
    frame = "map"

    timestamp = util_fb.createTimeStamp(builder, timeSec, timeNano)
    header = util_fb.createHeader(builder, timestamp, frame, projectUuid)

    childFrameId = builder.CreateString("camera")

    tfQuery = util_fb.createTransformStampedQuery(builder, header, childFrameId)
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
