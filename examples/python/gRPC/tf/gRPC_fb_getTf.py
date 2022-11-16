#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import TransformStamped, TransformStampedQuery
from fb import tf_service_grpc_fb as tfService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)
channel = util.get_gRPC_channel("local")
PROJECT_NAME = "simulatedDataWithInstances"
projectUuid = util_fb.getProject(builder, channel, PROJECT_NAME)

stubTf = tfService.TfServiceStub(channel)
builder = flatbuffers.Builder(1024)

timeSec = 1654688922
timeNano = 0
frame = "map"
header = util_fb.createHeader(builder, timeSec, timeNano, frame, projectUuid)

childFrameId = builder.CreateString("camera")

tfQuery = util_fb.createTransformStampedQuery(builder, header, childFrameId)
builder.Finish(tfQuery)

tfBuf = stubTf.GetTransformStamped(bytes(builder.Output()))

tf = TransformStamped.TransformStamped.GetRootAs(tfBuf)

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
