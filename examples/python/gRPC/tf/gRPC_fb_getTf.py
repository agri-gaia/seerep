#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import Header, Timestamp, TransformStamped, TransformStampedQuery
from fb import tf_service_grpc_fb as tfService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb


def createTimeStamp(builder, timeSec, timeNano):
    Timestamp.Start(builder)
    Timestamp.AddSeconds(builder, timeSec)
    Timestamp.AddNanos(builder, timeNano)
    return Timestamp.End(builder)


def createHeader(builder, timeSec, timeNano, frame, projectUuid):
    timeStamp = createTimeStamp(builder, timeSec, timeNano)

    frameId = builder.CreateString(frame)
    projectUuidStr = builder.CreateString(projectUuid)
    Header.Start(builder)
    Header.AddFrameId(builder, frameId)
    Header.AddStamp(builder, timeStamp)
    Header.AddUuidProject(builder, projectUuidStr)
    return Header.End(builder)


channel = util.get_gRPC_channel("local")
PROJECT_NAME = "simulatedDataWithInstances"
projectUuid = util_fb.get_or_create_project(channel, PROJECT_NAME, False)

stubTf = tfService.TfServiceStub(channel)
builder = flatbuffers.Builder(1024)

timeSec = 1654688922
timeNano = 0
frame = "map"
header = createHeader(builder, timeSec, timeNano, frame, projectUuid)

childFrameId = builder.CreateString("camera")

TransformStampedQuery.Start(builder)
TransformStampedQuery.AddChildFrameId(builder, childFrameId)
TransformStampedQuery.AddHeader(builder, header)
tfQuery = TransformStampedQuery.End(builder)
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
