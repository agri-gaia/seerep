#!/usr/bin/env python3
# NOTE: This file is referenced in the following mkdocs files:
#   writing-python-examples.md
# If any line changes on this file occur, those files may have to be updated as well

NUM_BB_LABELS = 5

import sys
import uuid
from typing import List, Optional, Tuple

import flatbuffers
import numpy as np
from grpc import Channel
from seerep.fb import BoundingBoxes2DLabeledStamped, Image, ProjectInfos
from seerep.fb import image_service_grpc_fb as imageService
from seerep.fb import meta_operations_grpc_fb as meta_ops
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createBoundingBox2dLabeledStamped,
    createBoundingBox2DLabeledWithCategory,
    createBoundingBoxes2d,
    createBoundingBoxes2dLabeled,
    createEmpty,
    createHeader,
    createLabelsWithInstance,
    createPoint2d,
    createQuery,
)


def add_bb_raw(
    target_proj_uuid: Optional[str] = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[Tuple[str, bytearray]]:
    stubMeta = meta_ops.MetaOperationsStub(grpc_channel)

    # 3. Check if we have an existing test project, if not, one is created.
    if target_proj_uuid is None:
        # 2. Get all projects from the server
        response = ProjectInfos.ProjectInfos.GetRootAs(
            stubMeta.GetProjects(bytes(createEmpty(flatbuffers.Builder(1024))))
        )
        for i in range(response.ProjectsLength()):
            proj_name = response.Projects(i).Name().decode()
            proj_uuid = response.Projects(i).Uuid().decode()
            print(proj_name + " " + proj_uuid)
            if proj_name == "testproject":
                target_proj_uuid = proj_uuid

        if target_proj_uuid is None:
            print("Please create a project with labeled images using gRPC_pb_sendLabeledImage.py first.")
            sys.exit()

    stub = imageService.ImageServiceStub(grpc_channel)

    builder = flatbuffers.Builder(1024)
    query = createQuery(builder, projectUuids=[builder.CreateString(target_proj_uuid)], withoutData=True)
    builder.Finish(query)
    buf = builder.Output()

    response_ls: List = list(stub.GetImage(bytes(buf)))
    if not response_ls:
        print("No images found. Please create a project with labeled images using gRPC_pb_sendLabeledImage.py first.")
        sys.exit()

    msgToSend = []
    bb_list: List[Tuple[str, bytearray]] = []

    for responseBuf in response_ls:
        response = Image.Image.GetRootAs(responseBuf)

        img_uuid = response.Header().UuidMsgs().decode("utf-8")
        header = createHeader(
            builder,
            projectUuid=response.Header().UuidProject().decode("utf-8"),
            msgUuid=img_uuid,
        )

        # create bounding box labels
        x1, y1 = np.random.rand(), np.random.rand()
        x2, y2 = np.random.rand(), np.random.rand()

        boundingBoxes = createBoundingBoxes2d(
            builder,
            [createPoint2d(builder, x1, y1) for _ in range(NUM_BB_LABELS)],
            [createPoint2d(builder, x2, y2) for _ in range(NUM_BB_LABELS)],
        )
        labelWithInstances = createLabelsWithInstance(
            builder,
            ["BoundingBoxLabel" + str(i) for i in range(NUM_BB_LABELS)],
            [str(uuid.uuid4()) for _ in range(NUM_BB_LABELS)],
            [float(1.0 / (i + 0.1)) for i in range(NUM_BB_LABELS)],
        )
        labelsBb = createBoundingBoxes2dLabeled(builder, labelWithInstances, boundingBoxes)

        boundingBox2DLabeledWithCategory = createBoundingBox2DLabeledWithCategory(
            builder, builder.CreateString("laterAddedBB"), labelsBb
        )

        labelsBbVector = createBoundingBox2dLabeledStamped(builder, header, [boundingBox2DLabeledWithCategory])
        builder.Finish(labelsBbVector)
        buf = builder.Output()

        bb_list.append(
            (
                img_uuid,
                buf,
            )
        )

        msgToSend.append(bytes(buf))

    stub.AddBoundingBoxes2dLabeled(iter(msgToSend))
    return bb_list


def add_bb(
    target_proj_uuid: Optional[str] = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[Tuple[str, BoundingBoxes2DLabeledStamped.BoundingBoxes2DLabeledStamped]]:
    return [
        (img_uuid, BoundingBoxes2DLabeledStamped.BoundingBoxes2DLabeledStamped.GetRootAs(bbbuf))
        for img_uuid, bbbuf in add_bb_raw(target_proj_uuid, grpc_channel)
    ]


if __name__ == "__main__":
    bb_list = add_bb()
    for img_uuid, bbs_img in bb_list:
        print(
            f"Added bounding boxes to image with uuid {img_uuid}, with the following center points and spatial extents:"
        )
        print("[center_point(x, y) | spatial_extent(x, y)]")
        for bbs_wcat in [bbs_img.LabelsBb(idx) for idx in range(bbs_img.LabelsBbLength())]:
            for bb in [
                bbs_wcat.BoundingBox2dLabeled(idx).BoundingBox() for idx in range(bbs_wcat.BoundingBox2dLabeledLength())
            ]:
                print(
                    f"[({bb.CenterPoint().X()}, {bb.CenterPoint().Y()}) | \
                    ({bb.SpatialExtent().X()}, {bb.SpatialExtent().Y()})]"
                )
        print()
