#!/usr/bin/env python3
NUM_BB_LABELS = 5

import os
import sys
import uuid

import flatbuffers
import numpy as np
from fb import Boundingbox, BoundingBoxes2DLabeledStamped, Image, Query
from fb import image_service_grpc_fb as imageService
from fb import meta_operations_grpc_fb as metaOperations

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)
channel = util.get_gRPC_channel("local")

projectuuid = util_fb.getProject(builder, channel, 'simulatedDataWithInstances')

if not projectuuid:
    exit

stub = imageService.ImageServiceStub(channel)

query = util_fb.createQuery(builder, projectUuids=[builder.CreateString(projectuuid)], withoutData=True)
builder.Finish(query)
buf = builder.Output()

msgToSend = []

for responseBuf in stub.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)

    header = util_fb.createHeader(
        builder,
        projectUuid=response.Header().UuidProject().decode("utf-8"),
        msgUuid=response.Header().UuidMsgs().decode("utf-8"),
    )

    # create bounding box labels
    x1, y1 = np.random.rand(), np.random.rand()
    x2, y2 = np.random.rand(), np.random.rand()

    print("SENDING ", x1, x2)
    print("SENDING ", y1, y2)

    boundingBoxes = util_fb.createBoundingBoxes2d(
        builder,
        [util_fb.createPoint2d(builder, x1, y1) for _ in range(NUM_BB_LABELS)],
        [util_fb.createPoint2d(builder, x2, y2) for _ in range(NUM_BB_LABELS)],
    )
    labelWithInstances = util_fb.createLabelsWithInstance(
        builder,
        ["BoundingBoxLabel" + str(i) for i in range(NUM_BB_LABELS)],
        [str(uuid.uuid4()) for _ in range(NUM_BB_LABELS)],
    )
    labelsBb = util_fb.createBoundingBoxes2dLabeled(builder, labelWithInstances, boundingBoxes)

    BoundingBoxes2DLabeledStamped.StartLabelsBbVector(builder, len(labelsBb))
    for labelBb in reversed(labelsBb):
        builder.PrependUOffsetTRelative(labelBb)
    labelsBbVector = builder.EndVector()

    labelsBbVector = util_fb.createBoundingBox2dLabeledStamped(builder, header, labelsBbVector)
    builder.Finish(labelsBbVector)
    buf = builder.Output()

    msgToSend.append(bytes(buf))


stub.AddBoundingBoxes2dLabeled(iter(msgToSend))
