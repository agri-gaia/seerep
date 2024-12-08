# test file for
#   gRPC_fb_sendImages.py
#   gRPC_fb_queryImages.py
from typing import Dict

import flatbuffers
import seerep.util.fb_helper as fbh
from gRPC.images import gRPC_fb_queryImages, gRPC_fb_sendImages
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def get_header_timestamp_nanos(fb_dict: Dict) -> int:
    return (
        fb_dict["header"]["stamp"]["seconds"] * 10**9
        + fb_dict["header"]["stamp"]["nanos"]
    )


def test_gRPC_fb_sendAndQueryImages(grpc_channel, project_setup, num_images=30):
    _, proj_uuid = project_setup

    camera_intrinsics_uuid = gRPC_fb_sendImages.send_cameraintrinsics(
        grpc_channel, proj_uuid
    )

    sent_images = gRPC_fb_sendImages.send_images(
        grpc_channel,
        proj_uuid,
        camera_intrinsics_uuid,
        gRPC_fb_sendImages.generate_image_ressources(num_images),
    )

    queried_images = gRPC_fb_queryImages.query_images_raw(
        flatbuffers.Builder(), grpc_channel, proj_uuid
    )

    assert queried_images is not None

    sent_images = [
        fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in sent_images
    ]

    queried_images = [
        fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in queried_images
    ]

    assert len(queried_images) == len(sent_images)

    for img in sent_images:
        assert img in queried_images


def test_gRPC_fb_sendAndQueryImagesEPSG32632(
    grpc_channel, epsg4326_project_setup, num_images=10
):
    proj_uuid = epsg4326_project_setup
    fbb = flatbuffers.Builder()

    camera_intrinsics_uuid = gRPC_fb_sendImages.send_cameraintrinsics(
        grpc_channel, proj_uuid
    )

    timestamp_nanos = 1245
    timestamps = [
        (t, timestamp_nanos)
        for t in range(1661336507, 1661336507 + num_images * 10, 10)
    ]

    sent_images = gRPC_fb_sendImages.send_images(
        grpc_channel,
        proj_uuid,
        camera_intrinsics_uuid,
        gRPC_fb_sendImages.generate_image_ressources(num_images),
        timestamps,
    )

    gRPC_fb_sendImages.send_tfs(grpc_channel, proj_uuid, timestamps)

    sent_images = [
        fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in sent_images
    ]

    offset_x = 394
    offset_y = 394
    # define polygon in epsg:32632
    polygon_epsg32632 = fbh.createRectangularPolygon2D(
        fbb,
        x=450945.55942398345 + offset_x,
        y=5801123.517354042 + offset_y,
        extent_x=517,
        extent_y=517,
        z=60,
        height=36,
    )

    queried_images = gRPC_fb_queryImages.query_images_raw(
        fbb,
        grpc_channel,
        proj_uuid,
        polygon2d=polygon_epsg32632,
        fullyEncapsulated=True,
        crsString="EPSG:32632",
    )

    queried_images = [
        fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in queried_images
    ]

    filtered_sent_imgs = sorted(
        sent_images, key=lambda d: get_header_timestamp_nanos(d)
    )
    sorted_queried_imgs = sorted(
        queried_images, key=lambda d: get_header_timestamp_nanos(d)
    )

    assert len(sorted_queried_imgs) > 0
    assert filtered_sent_imgs[2:4] == sorted_queried_imgs
