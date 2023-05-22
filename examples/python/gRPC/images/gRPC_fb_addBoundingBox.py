#!/usr/bin/env python3
NUM_BB_LABELS = 5

import uuid

import flatbuffers
import numpy as np
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createBoundingBox2dLabeledStamped,
    createBoundingBox2DLabeledWithCategory,
    createBoundingBoxes2d,
    createBoundingBoxes2dLabeled,
    createHeader,
    createLabelsWithInstance,
    createPoint2d,
    createQuery,
    getProject,
)

builder = flatbuffers.Builder(1024)
channel = get_gRPC_channel("local")

projectuuid = getProject(builder, channel, 'testproject')

if not projectuuid:
    exit()

stub = imageService.ImageServiceStub(channel)

query = createQuery(builder, projectUuids=[builder.CreateString(projectuuid)], withoutData=True)
builder.Finish(query)
buf = builder.Output()

msgToSend = []

for responseBuf in stub.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)

    header = createHeader(
        builder,
        projectUuid=response.Header().UuidProject().decode("utf-8"),
        msgUuid=response.Header().UuidMsgs().decode("utf-8"),
    )

    # create bounding box labels
    x1, y1 = np.random.rand(), np.random.rand()
    x2, y2 = np.random.rand(), np.random.rand()

    print("SENDING x1, y1: ", x1, " , ", y1)
    print("SENDING x2, y2: ", x2, " , ", y2)

    boundingBoxes = createBoundingBoxes2d(
        builder,
        [createPoint2d(builder, x1, y1) for _ in range(NUM_BB_LABELS)],
        [createPoint2d(builder, x2, y2) for _ in range(NUM_BB_LABELS)],
    )
    labelWithInstances = createLabelsWithInstance(
        builder,
        ["BoundingBoxLabel" + str(i) for i in range(NUM_BB_LABELS)],
        [1.0 / (i + 0.1) for i in range(NUM_BB_LABELS)],
        [str(uuid.uuid4()) for _ in range(NUM_BB_LABELS)],
    )
    labelsBb = createBoundingBoxes2dLabeled(builder, labelWithInstances, boundingBoxes)

    boundingBox2DLabeledWithCategory = createBoundingBox2DLabeledWithCategory(
        builder, builder.CreateString("laterAddedBB"), labelsBb
    )

    labelsBbVector = createBoundingBox2dLabeledStamped(builder, header, [boundingBox2DLabeledWithCategory])
    builder.Finish(labelsBbVector)
    buf = builder.Output()

    msgToSend.append(bytes(buf))


stub.AddBoundingBoxes2dLabeled(iter(msgToSend))
