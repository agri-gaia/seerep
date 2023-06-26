# test file for
#   gRPC_pb_queryImage.py
#   gRPC_pb_sendLabeledImage.py

import os
import sys

script_path = os.path.realpath(os.path.dirname(__name__))
os.chdir(script_path)
sys.path.append("../../..")
# above mentioned steps will make 1 level up module available for import
from gRPC.images import gRPC_pb_queryImage as query_img
from gRPC.images import gRPC_pb_sendLabeledImage as send_img
from seerep.pb import image_service_pb2 as image

from examples.python.gRPC import seerep_python_examples


# test sending and querying the images
def test_gRPC_pb_sendLabeledImage_queryImage():
    sent_image_list: image.Image = send_img.send_labeled_images()

    queried_image_list: image.Image = query_img.query_images()

    assert len(queried_image_list) == 4

    for img in queried_image_list:
        assert img in sent_image_list
