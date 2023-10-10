#!/usr/bin/env python3
NUM_BB_LABELS = 5

import uuid

import flatbuffers
import numpy as np
from seerep.fb import PointCloud2
from seerep.fb import point_cloud_service_grpc_fb as pointcloudService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createBoundingBoxes,
    createBoundingBoxesLabeled,
    createBoundingBoxLabeledStamped,
    createBoundingBoxLabeledWithCategory,
    createHeader,
    createLabelsWithInstance,
    createPoint,
    createQuery,
    getProject,
)

builder = flatbuffers.Builder(1024)
channel = get_gRPC_channel()

projectuuid = getProject(builder, channel, 'testproject')

if not projectuuid:
    print("Requested project does not exist")
    exit()

stub = pointcloudService.PointCloudServiceStub(channel)

query = createQuery(builder, projectUuids=[builder.CreateString(projectuuid)], withoutData=True)
builder.Finish(query)
buf = builder.Output()

msgToSend = []

for responseBuf in stub.GetPointCloud2(bytes(buf)):
    response = PointCloud2.PointCloud2.GetRootAs(responseBuf)

    header = createHeader(
        builder,
        projectUuid=response.Header().UuidProject().decode("utf-8"),
        msgUuid=response.Header().UuidMsgs().decode("utf-8"),
    )

    # create bounding box labels
    x1, y1, z1 = np.random.rand(3)
    x2, y2, z2 = np.random.rand(3)

    print("SENDING x1, y1, z1: ", x1, " , ", y1, " , ", z1)
    print("SENDING x2, y2, z2: ", x2, " , ", y2, " , ", z2)

    boundingBoxes = createBoundingBoxes(
        builder,
        [createPoint(builder, x1, y1, z1) for _ in range(NUM_BB_LABELS)],
        [createPoint(builder, x2, y2, z2) for _ in range(NUM_BB_LABELS)],
    )
    labelWithInstances = createLabelsWithInstance(
        builder,
        ["BoundingBoxLabel" + str(i) for i in range(NUM_BB_LABELS)],
        [1.0 / (i + 0.1) for i in range(NUM_BB_LABELS)],
        [str(uuid.uuid4()) for _ in range(NUM_BB_LABELS)],
    )
    labelsBb = createBoundingBoxesLabeled(builder, labelWithInstances, boundingBoxes)

    boundingBoxLabeledWithCategory = createBoundingBoxLabeledWithCategory(
        builder, builder.CreateString("laterAddedBB"), labelsBb
    )

    labelsBbVector = createBoundingBoxLabeledStamped(builder, header, [boundingBoxLabeledWithCategory])
    builder.Finish(labelsBbVector)
    buf = builder.Output()

    msgToSend.append(bytes(buf))


stub.AddBoundingBoxesLabeled(iter(msgToSend))
