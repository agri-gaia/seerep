# test file for
#  gRPC_fb_addBoundingBox.py

from typing import List, cast

import conftest
import flatbuffers
from grpc import Channel
from gRPC.images import gRPC_fb_addBoundingBox as add_bb
from gRPC.images import gRPC_pb_sendLabeledImage as add_img
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.util.fb_helper import createQuery


def get_imgs(target_proj_uuid: str, grpc_channel: Channel) -> List:

    builder = flatbuffers.Builder(1024)
    stub = imageService.ImageServiceStub(grpc_channel)
    query = createQuery(
        builder, projectUuids=[builder.CreateString(target_proj_uuid)], withoutData=True
    )
    builder.Finish(query)
    buf = builder.Output()
    response_ls: List = list(stub.GetImage(bytes(buf)))

    return [Image.Image.GetRootAs(buf) for buf in response_ls]


def test_addBoundingBox(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    # send labeled images to the server for preparation
    add_img.send_labeled_images(target_proj_uuid=proj_uuid, grpc_channel=grpc_channel)

    sent_bb = add_bb.add_bb(target_proj_uuid=proj_uuid, grpc_channel=grpc_channel)
    assert sent_bb is not None

    # use a regular query to query the images from the server to check if the bounding box is the same
    all_imgs = get_imgs(proj_uuid, grpc_channel)

    for bb_img_uuid, bbs_img in sent_bb:
        img_bb = [
            img
            for img in all_imgs
            if img.Header().UuidMsgs().decode("utf-8") == bb_img_uuid
        ][0]
        # iterate through all categories of the image
        filtered_bbs = [
            img_bb.LabelsBb(idx)
            for idx in range(img_bb.LabelsBbLength())
            if img_bb.LabelsBb(idx).Category().decode("utf-8") == "laterAddedBB"
        ]
        sent_bbs = [bbs_img.LabelsBb(idx) for idx in range(bbs_img.LabelsBbLength())]

        assert len(filtered_bbs) == len(sent_bbs)
        for bb_cat in filtered_bbs:
            assert conftest.fb_obj_to_dict(bb_cat) in [
                conftest.fb_obj_to_dict(bb) for bb in sent_bbs
            ]
