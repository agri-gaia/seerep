# test file for
#   gRPC_fb_addImage.py
#   gRPC_fb_queryImages.py

from gRPC.images import gRPC_fb_addImage as add_image
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def test_gRPC_fb_addAndQueryImages(grpc_channel, project_setup, num_images=10):
    _, proj_uuid = project_setup

    camera_intrinsics_uuid = add_image.send_cameraintrinsics(
        grpc_channel, proj_uuid
    )

    # test for image uuid, general label and the general label confidence
    sent_images = add_image.add_images(
        grpc_channel,
        proj_uuid,
        camera_intrinsics_uuid,
        add_image.generate_image_storage(num_images),
    )

    queried_images = add_image.get_image(grpc_channel, proj_uuid)

    assert queried_images is not None

    sent_images = sorted(
        [fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in sent_images],
        key=lambda img: img["header"]["uuid_msgs"],
    )

    queried_images = sorted(
        [fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in queried_images],
        key=lambda img: img["header"]["uuid_msgs"],
    )

    assert len(queried_images) == len(sent_images)

    for img in sent_images:
        assert img in queried_images
