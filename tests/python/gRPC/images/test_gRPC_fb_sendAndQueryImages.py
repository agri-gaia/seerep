# test file for
#   gRPC_fb_sendImages.py
#   gRPC_fb_queryImages.py


import flatbuffers
from gRPC.images import gRPC_fb_queryImages, gRPC_fb_sendImages
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


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
