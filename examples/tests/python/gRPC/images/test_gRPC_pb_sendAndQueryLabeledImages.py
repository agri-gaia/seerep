# test file for
#   gRPC_pb_queryImage.py
#   gRPC_pb_sendLabeledImage.py
import logging
from typing import List

from gRPC.images import gRPC_pb_queryImage as query_img
from gRPC.images import gRPC_pb_sendLabeledImage as send_img
from seerep.pb import image_service_pb2 as image


# test sending and querying the images
def test_gRPC_pb_sendAndQueryImages(grpc_channel, project_setup):

    proj_name, proj_uuid = project_setup
    logging.info(f"Testing project: {proj_name}; {proj_uuid}")

    # forcing new project for reproducibility
    sent_image_list: List[image.Image] = send_img.send_labeled_images(
        grpc_channel, proj_uuid
    )

    queried_image_list: List[image.Image] = query_img.query_images(
        grpc_channel, proj_uuid
    )

    # 10 images are sent
    # the query is constraint to request 4 based on their attributes
    assert len(sent_image_list) == 10
    assert len(queried_image_list) == 4

    for img in queried_image_list:
        assert img in sent_image_list
