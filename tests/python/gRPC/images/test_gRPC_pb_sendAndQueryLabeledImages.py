# test file for
#   gRPC_pb_queryImage.py
#   gRPC_pb_sendLabeledImage.py
import logging
from typing import List

from gRPC.images import gRPC_pb_queryImage as query_img
from gRPC.images import gRPC_pb_sendLabeledImage as send_img
from seerep.pb import image_pb2 as image


# test sending and querying the images
def test_gRPC_pb_sendAndQueryImages(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    sent_images = []
    # this adds the servers image uuid to the images and a instance uuid which
    # is given by the server aswell
    for img in send_img.send_labeled_images(proj_uuid, grpc_channel)[0]:
        completed_img = img[1]
        completed_img.header.uuid_msgs = img[0]
        sent_images.append(completed_img)

    logging.info(f"Sending images to project: {proj_name}; {proj_uuid}")

    queried_image_list: List[image.Image] = query_img.query_images(
        proj_uuid, grpc_channel
    )

    # 10 images are sent
    # the query is constraint to request 8 based on their attributes
    assert len(sent_images) == 10
    assert len(queried_image_list) == 8

    for img in queried_image_list:
        assert img in sent_images
