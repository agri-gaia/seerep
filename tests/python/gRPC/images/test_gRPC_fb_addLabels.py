# NOTE: This file is referenced in the following mkdocs files:
#   writing-python-tests.md
# If any line changes on this file occur, those files may have to be updated as well
# test file for
#   gRPC_fb_addBoundingBox.py
# requires:
#   gRPC_pb_sendLabeledImage.py

from typing import List

import flatbuffers
from grpc import Channel
from gRPC.images import gRPC_fb_addLabel as add_label
from gRPC.images import gRPC_pb_sendLabeledImage as add_img
from seerep.fb import image_service_grpc_fb as imageService
from seerep.util.fb_helper import createQuery
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def get_imgs(target_proj_uuid: str, grpc_channel: Channel) -> List:
    builder = flatbuffers.Builder(1024)
    stub = imageService.ImageServiceStub(grpc_channel)
    query = createQuery(builder, projectUuids=[builder.CreateString(target_proj_uuid)], withoutData=True)
    builder.Finish(query)
    buf = builder.Output()
    response_ls: List = list(stub.GetImage(bytes(buf)))

    return [fb_flatc_dict(buf, SchemaFileNames.IMAGE) for buf in response_ls]


def test_addLabel(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    # send labeled images to the server for preparation
    add_img.send_labeled_images(target_proj_uuid=proj_uuid, grpc_channel=grpc_channel)

    sent_label = add_label.add_label_raw(target_proj_uuid=proj_uuid, grpc_channel=grpc_channel)

    assert sent_label is not None

    # use a regular query to query the images from the server to check if the label is the same
    all_imgs = get_imgs(proj_uuid, grpc_channel)

    for label_img_uuid, label_img in sent_label:
        img_label = [img for img in all_imgs if img["header"]["uuid_msgs"] == label_img_uuid][0]

        # iterate through all categories of the image
        filtered_label = [label for label in img_label["labels"] if label["category"] == "laterAddedLabel"]

        sent_labels = fb_flatc_dict(label_img, SchemaFileNames.DATASET_UUID_LABEL)["labels"]

        assert len(filtered_label) == len(sent_labels)

        for label_cat in filtered_label:
            assert label_cat in sent_labels
