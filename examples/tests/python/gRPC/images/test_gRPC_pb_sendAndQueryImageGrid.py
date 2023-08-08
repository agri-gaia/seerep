# test file for
#   gRPC_pb_queryImage.py
#   gRPC_pb_sendLabeledImage.py
import logging

from gRPC.images import gRPC_pb_queryImageGrid as query_grid
from gRPC.images import gRPC_pb_sendLabeledImageGrid as send_grid


# test sending and querying the images
def test_gRPC_pb_sendAndQueryImageGrid(grpc_channel, project_setup):

    proj_name, proj_uuid = project_setup
    logging.info(f"Testing project: {proj_name}; {proj_uuid}")

    # test for image uuid, general label and the general label confidence of corresponding entry in 2d array
    sent_grid = send_grid.send_labeled_image_grid(proj_uuid, grpc_channel)
    sent_test_info = [
        [
            [
                (
                    uuid,
                    img.labels_general[0].labelWithInstance[1].label.label,
                    img.labels_general[0].labelWithInstance[1].label.confidence,
                )
                for uuid, img in img_lst
            ]
            for img_lst in inner_lst
        ]
        for inner_lst in sent_grid
    ]

    # querying the images should yield the same images in the same array x, y index
    queried_grid = query_grid.query_image_grid(proj_uuid, grpc_channel)

    for x in range(len(sent_test_info)):
        for y in range(len(sent_test_info[0])):
            sent_imgs_gridpos = sent_test_info[x][y]
            query_imgs_gridpos = queried_grid[x][y]
            assert len(sent_imgs_gridpos) == len(query_imgs_gridpos)
            # sort both lists by their uuid
            sent_imgs_gridpos = sorted(sent_imgs_gridpos, key=lambda i: i[0])
            query_imgs_gridpos = sorted(
                query_imgs_gridpos, key=lambda i: i.header.uuid_msgs
            )
            # compare their relevant contents
            for idx, img in enumerate(query_imgs_gridpos):
                assert sent_imgs_gridpos[idx][0] == img.header.uuid_msgs
                assert (
                    sent_imgs_gridpos[idx][1]
                    == img.labels_general[0].labelWithInstance[1].label.label
                )
                assert (
                    sent_imgs_gridpos[idx][2]
                    == img.labels_general[0].labelWithInstance[1].label.confidence
                )
