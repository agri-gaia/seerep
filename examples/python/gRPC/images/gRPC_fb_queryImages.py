#!/usr/bin/env python3
# NOTE: This file is referenced in the following mkdocs files:
#   images.md
# Any changes done in here will be reflected in there
import sys
from typing import List

import flatbuffers
from grpc import Channel
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    create_label,
    create_label_category,
    createQuery,
    createRectangularPolygon2D,
    createTimeInterval,
    createTimeStamp,
    getOrCreateProject,
)


def query_images_raw(
    fbb: flatbuffers.Builder,
    grpc_channel: Channel,
    target_proj_uuid: str,
    *args,
    **kwargs,
) -> List[bytearray]:
    """
    Query images from SEEREP and return the raw FlatBuffer buffers.

    Args:
        grpc_channel (Channel): gRPC channel to communicate with the server.
        target_proj_uuid (str): UUID of the target project.
        *args: Variable args to pass to the query.
        **kwargs: Keyword args to pass to the query.

    Returns:
        List[bytearray]: A list buffers of images matching the query.
    """
    image_service_stub = imageService.ImageServiceStub(grpc_channel)

    query = createQuery(fbb, *args, projectUuids=[target_proj_uuid], **kwargs)

    fbb.Finish(query)
    query_buf = fbb.Output()

    return image_service_stub.GetImage(bytes(query_buf))


def query_images(
    fbb: flatbuffers.Builder,
    grpc_channel: Channel,
    target_proj_uuid: str,
    *args,
    **kwargs,
) -> List[Image.Image]:
    """
    Query images from SEEREP.

    Args:
        grpc_channel (Channel): gRPC channel to communicate with the server.
        target_proj_uuid (str): The UUID of the target project.
        *args: Variable args to pass to the query.
        **kwargs: Keyword args to pass to the query.

    Returns:
        List[Image.Image]: A list of images matching the query.
    """
    return [
        Image.Image.GetRootAs(img)
        for img in query_images_raw(
            fbb, grpc_channel, target_proj_uuid, *args, **kwargs
        )
    ]


def query_example():
    fbb = flatbuffers.Builder()
    grpc_channel = get_gRPC_channel()
    project_uuid = getOrCreateProject(fbb, grpc_channel, "testproject")

    polygon_2d = createRectangularPolygon2D(
        fbb,
        x=-100,
        extent_x=200,
        extent_y=200,
        y=-100,
        z=-100,
        height=700,
    )

    time_min = createTimeStamp(fbb, 1610549273, 0)
    time_max = createTimeStamp(fbb, 1938549273, 0)
    time_interval = createTimeInterval(fbb, time_min, time_max)  # noqa: F841

    project_uuids = [project_uuid]  # noqa: F841

    labels = [
        create_label(builder=fbb, label=label_str, label_id=i)
        for i, label_str in enumerate(["label1", "label2"])
    ]

    labelsCategory = [
        create_label_category(
            builder=fbb,
            labels=labels,
            datumaro_json="a very valid datumaro json",
            category="category A",
        )
    ]

    matching_images = query_images(
        fbb,
        grpc_channel,
        project_uuid,
        # polygon2d=polygon_2d,
        polygon2dSensorPos=polygon_2d,
        labels=labelsCategory,
        withoutData=True,
        fullyEncapsulated=False,
    )

    if matching_images is None or len(matching_images) == 0:
        print("No images matched the query.")
        sys.exit()

    print(f"Number of images matching the query: {len(matching_images)}")

    for img in matching_images:
        print(
            "------------------------------------------------------------------"
        )
        print(f"Msg UUID: {img.Header().UuidMsgs().decode('utf-8')}")
        print(f"Number of labels: {img.LabelsLength()}")
        if img.LabelsLength() > 0:
            print(
                "First label: "
                + img.Labels(0).Labels(0).Label().decode("utf-8")
            )
    print("------------------------------------------------------------------")


def query_geodetic_example():
    fbb = flatbuffers.Builder()
    grpc_channel = get_gRPC_channel()
    project_uuid = getOrCreateProject(fbb, grpc_channel, "geodeticProject")

    polygon_epsg3857 = createRectangularPolygon2D(
        fbb,
        x=921689.630348,
        y=6865153.476919,
        extent_x=655,
        extent_y=655,
        z=0,
        height=100,
    )

    matching_images = query_images(
        fbb,
        grpc_channel,
        project_uuid,
        polygon2d=polygon_epsg3857,
        withoutData=True,
        fullyEncapsulated=False,
        crsString="EPSG:3857",
    )

    if matching_images is None or len(matching_images) == 0:
        print("No images matched the query.")
        sys.exit()

    print(f"Number of images matching the query: {len(matching_images)}")

    for img in matching_images:
        print(
            "------------------------------------------------------------------"
        )
        print(f"Msg UUID: {img.Header().UuidMsgs().decode('utf-8')}")
        print(f"Number of labels: {img.LabelsLength()}")
        if img.LabelsLength() > 0:
            print(
                "First label: "
                + img.Labels(0).Labels(0).Label().decode("utf-8")
            )
    print("------------------------------------------------------------------")


if __name__ == "__main__":
    query_example()
    # query_geodetic_example()
