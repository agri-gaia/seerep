# test file for
#   gRPC_pb_queryImage.py
#   gRPC_pb_sendLabeledImage.py
import logging
from typing import List, Tuple

from gRPC.images import gRPC_pb_queryImage as query_img
from gRPC.images import gRPC_pb_sendLabeledImage as send_img
from seerep.pb import image_pb2 as image

# send images to server
# seperated so that server is restarted
# @pytest.fixture
# def send_images(grpc_channel, project_setup) -> Tuple[List[image.Image], Tuple[str, str]]:
#     proj_name, proj_uuid = project_setup
#     logging.info(f"Sending images to project: {proj_name}; {proj_uuid}")

# test sending and querying the images
def test_gRPC_pb_sendAndQueryImages(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    send_images = [
        img[1] for img in send_img.send_labeled_images(proj_uuid, grpc_channel)
    ]

    logging.info(f"Sending images to project: {proj_name}; {proj_uuid}")

    queried_image_list: List[image.Image] = query_img.query_images(
        proj_uuid, grpc_channel
    )

    # 10 images are sent
    # the query is constraint to request 4 based on their attributes
    assert len(send_images) == 10
    assert len(queried_image_list) == 4

    for img in queried_image_list:
        assert img in send_images
