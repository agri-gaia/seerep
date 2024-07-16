#!/usr/bin/env python3
# NOTE: This file is referenced in the following mkdocs files:
#   images.md
# Any changes done in here will be reflected in there
import sys
from typing import List, Optional

import flatbuffers
from grpc import Channel
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    create_label,
    create_label_category,
    createPoint2d,
    createPolygon2D,
    createQuery,
    createTimeInterval,
    createTimeStamp,
    getProject,
)


def query_images_raw(
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> List[bytearray]:
    builder = flatbuffers.Builder(1024)
    if target_proj_uuid is None:
        # 1. Get all projects from the server
        target_proj_uuid = getProject(builder, grpc_channel, "testproject")
        # 2. Check if the defined project exist, else return None
        if target_proj_uuid is None:
            print("""
                valid project doesn't exist! Please execute
                gRPC_fb_addCameraIntrinsics.py beforehand.
            """)
            return None

    # 3. Get gRPC service object
    stub = imageService.ImageServiceStub(grpc_channel)

    # Create all necessary objects for the query
    scale = 100
    vertices = [
        createPoint2d(builder, x * scale, y * scale)
        for x, y in [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)]
    ]
    polygon2d = createPolygon2D(builder, 700, -100, vertices)

    # ruff: noqa: F841
    timeMin = createTimeStamp(builder, 1610549273, 0)
    timeMax = createTimeStamp(builder, 1938549273, 0)
    timeInterval = createTimeInterval(builder, timeMin, timeMax)

    projectUuids = [builder.CreateString(target_proj_uuid)]

    labelStr = ["label1", "label2"]
    labels = []
    for labelAct in labelStr:
        labels.append(create_label(builder=builder, label=labelAct, label_id=1))
    labelsCategory = []
    labelsCategory.append(
        create_label_category(
            builder=builder,
            labels=labels,
            datumaro_json="a very valid datumaro json",
            category="category A",
        )
    )

    dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
    instanceUuids = [
        builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")
    ]

    # 4. Create a query with parameters
    # All parameters are optional!
    # With all parameters set (especially with the data and instance uuids set)
    # the result of the query will be empty.
    # Set the query parameters to adequate values or remove them from the query
    # creation
    query = createQuery(
        builder,
        polygon2d=polygon2d,
        # timeInterval=timeInterval,
        labels=labelsCategory,
        # mustHaveAllLabels=True,
        projectUuids=projectUuids,
        # instanceUuids=instanceUuids,
        # dataUuids=dataUuids,
        withoutData=True,
        fullyEncapsulated=False,
        inMapFrame=True,
    )
    builder.Finish(query)
    buf = builder.Output()

    # 5. Query the server for images matching the query and iterate over them
    queried_images = stub.GetImage(bytes(buf))

    return queried_images


def query_images(
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> List[Image.Image]:
    return [
        Image.Image.GetRootAs(img)
        for img in query_images_raw(target_proj_uuid, grpc_channel)
    ]


if __name__ == "__main__":
    queried_images = query_images()
    if queried_images is None:
        print("no images were queried")
        sys.exit()
    print(f"count of images: {len(queried_images)}")

    for img in queried_images:
        print(
            "------------------------------------------------------------------"
        )
        print(f"uuidmsg: {img.Header().UuidMsgs().decode('utf-8')}")
        print(f"count of bounding box labels: {img.LabelsLength()}")
        if img.LabelsLength() > 0:
            print(
                "first label: "
                + img.Labels(0).Labels(0).Label().decode("utf-8")
            )
    print("------------------------------------------------------------------")
