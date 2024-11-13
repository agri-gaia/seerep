# test file for
#   gRPC_fb_queryImage.py
#   gRPC_pb_sendLabeledImage.py

import flatbuffers
from boltons.iterutils import remap
from google.protobuf import json_format
from gRPC.images import gRPC_fb_queryImages
from gRPC.images import gRPC_pb_sendLabeledImage as send_img
from seerep.util.common import remap_keys, remap_to_snake_case, trunc_floats
from seerep.util.fb_helper import (
    create_label,
    create_label_category,
    createPoint2d,
    createPolygon2D,
)
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def _image_dict_pipeline(p, k, v):
    mapping_keys = {
        "bounding_box2_d_labeled": "bounding_box2d_labeled",
        "uuid_camera_intrinsics": "uuid_cameraintrinsics",
        "labels_with_instance": "label_with_instance",
    }

    k, v = trunc_floats(p, k, v, ignore_after=10)
    k, v = remap_to_snake_case(p, k, v)
    k, v = remap_keys(p, k, v, mapping_keys)
    return k, v


# test sending and querying the images
def test_gRPC_fb_queryImages(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup
    fbb = flatbuffers.Builder()

    sent_images = []

    # this adds the servers image uuid to the images and a instance uuid which
    # is given by the server aswell
    for img in send_img.send_labeled_images(proj_uuid, grpc_channel)[0]:
        completed_img = img[1]
        completed_img.header.uuid_msgs = img[0]
        sent_images.append(completed_img)

    print(f"Sending images to project: {proj_name}; {proj_uuid}")

    # create the data for the query
    scale = 100
    vertices = [
        createPoint2d(fbb, x * scale, y * scale)
        for x, y in [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)]
    ]

    polygon_2d = createPolygon2D(fbb, 700, -100, vertices)

    labels = [
        create_label(builder=fbb, label=label_str, label_id=1)
        for label_str in ["label1", "label2"]
    ]

    labelsCategory = [
        create_label_category(
            builder=fbb,
            labels=labels,
            datumaro_json="a very valid datumaro json",
            category="category A",
        )
    ]

    matching_images = gRPC_fb_queryImages.query_images_raw(
        fbb,
        grpc_channel,
        proj_uuid,
        polygon2d=polygon_2d,
        labels=labelsCategory,
        withoutData=True,
        fullyEncapsulated=False,
        crsString="map",
    )

    queried_image_dicts = [
        remap(
            fb_flatc_dict(img, SchemaFileNames.IMAGE),
            visit=_image_dict_pipeline,
        )
        for img in matching_images
    ]
    sent_image_dicts = [
        remap(json_format.MessageToDict(img), visit=_image_dict_pipeline)
        for img in sent_images
    ]

    # replace data field with because in the query withoutData is set to true
    for img in sent_image_dicts:
        img.pop("data")

    # filter sent_images to only contain the 6 images which are queried with
    # uuid_msgs
    filtered_sent_images = []
    for img in sent_image_dicts:
        if img["header"]["uuid_msgs"] in [
            img["header"]["uuid_msgs"] for img in queried_image_dicts
        ]:
            filtered_sent_images.append(img)

    assert len(queried_image_dicts) == len(filtered_sent_images) == 6

    ordered_sent_imgs = sorted(
        filtered_sent_images, key=lambda x: x["header"]["uuid_msgs"]
    )
    ordered_queried_imgs = sorted(
        queried_image_dicts, key=lambda x: x["header"]["uuid_msgs"]
    )

    assert ordered_queried_imgs == ordered_sent_imgs
