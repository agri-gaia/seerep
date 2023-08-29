# test file for
#   gRPC_fb_queryImage.py
from typing import List

from gRPC.images import gRPC_fb_queryImage as query_img
from gRPC.images import gRPC_pb_sendLabeledImage as send_img
from seerep.fb import Image


# test sending and querying the images
def test_gRPC_fb_queryImages(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    sent_images = []
    # this adds the servers image uuid to the images and a instance uuid which is given by the server aswell
    for img in send_img.send_labeled_images(proj_uuid, grpc_channel):
        completed_img = img[1]
        completed_img.header.uuid_msgs = img[0]
        for category in completed_img.labels_general:
            for instance in category.labelWithInstance:
                instance.instanceUuid = "00000000-0000-0000-0000-000000000000"
        sent_images.append(completed_img)

    print(f"Sending images to project: {proj_name}; {proj_uuid}")

    queried_image_list: List[Image.Image] = query_img.query_images(
        proj_uuid, grpc_channel
    )

    # 10 images are sent
    # the query is constraint to request 5 based on their attributes
    assert len(sent_images) == 10
    assert len(queried_image_list) == 5

    print(dir(sent_images[0].DESCRIPTOR))

    assert False
    for img in queried_image_list:
        assert img in sent_images
