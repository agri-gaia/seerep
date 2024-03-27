#!/usr/bin/env python3
NUM_BB_LABELS = 5

import sys
import uuid
from typing import List

import flatbuffers
import numpy as np
from grpc import Channel
from seerep.fb import BoundingBoxesLabeledStamped, PointCloud2
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


def add_pc_bounding_boxes_raw(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[bytearray]:
    builder = flatbuffers.Builder(1024)

    if target_proj_uuid is None:
        target_proj_uuid = getProject(builder, grpc_channel, "testproject")

        if target_proj_uuid is None:
            print("Requested project does not exist")
            sys.exit()

    stub = pointcloudService.PointCloudServiceStub(grpc_channel)

    query = createQuery(builder, projectUuids=[builder.CreateString(target_proj_uuid)], withoutData=True)
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

    response = stub.AddBoundingBoxesLabeled(iter(msgToSend))
    return msgToSend


def add_pc_bounding_boxes(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[BoundingBoxesLabeledStamped.BoundingBoxesLabeledStamped]:
    return [
        BoundingBoxesLabeledStamped.BoundingBoxesLabeledStamped.GetRootAs(resp_buf)
        for resp_buf in add_pc_bounding_boxes_raw(target_proj_uuid, grpc_channel)
    ]


if __name__ == "__main__":
    bbs_lst: List[BoundingBoxesLabeledStamped.BoundingBoxesLabeledStamped] = add_pc_bounding_boxes()

    for bbs in bbs_lst:
        for label_idx in range(bbs.LabelsBbLength()):
            for bb_idx in range(bbs.LabelsBb(label_idx).BoundingBoxLabeledLength()):
                header_uuid = bbs.Header().UuidMsgs().decode()
                print(f"uuid: {header_uuid}")
                cp = bbs.LabelsBb(label_idx).BoundingBoxLabeled(bb_idx).BoundingBox().CenterPoint()
                se = bbs.LabelsBb(label_idx).BoundingBoxLabeled(bb_idx).BoundingBox().SpatialExtent()

                print(f"    SENT CENTER POINT (x1, y1, z1): ({cp.X()}, {cp.Y()}, {cp.Z()})")
                print(f"    SENT SPATIAL EXTENT (x1, y1, z1): ({se.X()}, {se.Y()}, {se.Z()})")
