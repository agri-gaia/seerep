# test file for
#   gRPC_fb_queryImage.py
#   gRPC_pb_sendLabeledImage.py
from typing import List

import fb_to_dict
import pb_to_dict
from gRPC.images import gRPC_fb_queryImage as query_img
from gRPC.images import gRPC_pb_sendLabeledImage as send_img
from seerep.fb import Image


# test sending and querying the images
def test_gRPC_fb_queryImages(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    sent_images = []

    # this adds the servers image uuid to the images and a instance uuid which is given by the server aswell
    for img in send_img.send_labeled_images(proj_uuid, grpc_channel)[0]:
        completed_img = img[1]
        completed_img.header.uuid_msgs = img[0]
        for category in completed_img.labels_general:
            for instance in category.labelWithInstance:
                instance.instanceUuid = "00000000-0000-0000-0000-000000000000"
        sent_images.append(completed_img)

    print(f"Sending images to project: {proj_name}; {proj_uuid}")

    # print(f"img:\n {pb_to_dict.pb_to_dict(sent_images[0])['header']}")
    fb_to_pb_keys = {
        "uuid_cameraintrinsics": "uuid_camera_intrinsics",
        "bounding_box2d_labeled": "bounding_box2_d_labeled",
        "labels_with_instance": "label_with_instance",
    }

    queried_image_list: List[Image.Image] = query_img.query_images(
        proj_uuid, grpc_channel
    )

    queried_image_dicts = [
        fb_to_dict.fb_obj_to_dict(img, True, fb_to_pb_keys)
        for img in queried_image_list
    ]
    sent_image_dicts = [pb_to_dict.pb_to_dict(img, True) for img in sent_images]

    # replace data field with because in the query withoutData is set to true
    for img in sent_image_dicts:
        img["data"] = []

    # filter sent_images to only contain the 6 images which are queried with uuid_msgs
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
