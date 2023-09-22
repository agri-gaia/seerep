# test file for
#   gRPC_pb_queryImageGrid.py
#   gRPC_pb_sendLabeledImageGrid.py
import logging

from gRPC.images import gRPC_pb_queryImageGrid as query_grid
from gRPC.images import gRPC_pb_sendLabeledImageGrid as send_grid


# test sending and querying the images
def test_gRPC_pb_sendAndQueryImageGrid(grpc_channel, project_setup):

    proj_name, proj_uuid = project_setup
    logging.info(f"Testing project: {proj_name}; {proj_uuid}")

    # test for image uuid, general label and the general label confidence of corresponding entry in 2d array
    sent_images = []
    # this adds the servers image uuid to the images and a instance uuid which is given by the server aswell
    sent_images_grid, _, _ = send_grid.send_labeled_image_grid(proj_uuid, grpc_channel)

    for x in range(len(sent_images_grid)):
        sent_images.append([])
        for y in range(len(sent_images_grid[x])):
            sent_images[x].append([])
            for i, img in enumerate(sent_images_grid[x][y]):
                completed_img = sent_images_grid[x][y][i][1]
                completed_img.header.uuid_msgs = sent_images_grid[x][y][i][0]
                sent_images[x][y].append(completed_img)

    # flatten sent list
    sent_images_flattened = [
        img for lst in sent_images for inner_lst in lst for img in inner_lst
    ]

    # querying the images should yield the same images in the same array x, y index
    queried_grid = query_grid.query_image_grid(proj_uuid, grpc_channel)

    queried_imgs_flattened = [
        img for lst in queried_grid for inner_lst in lst for img in inner_lst
    ]

    # check if all images that have been queried are in the sent images
    for img in queried_imgs_flattened:
        assert img in sent_images_flattened

    # check if the grid layout is correct
    for x in range(len(sent_images)):
        for y in range(len(sent_images[x])):
            sent_imgs_gridpos = sent_images[x][y]
            query_imgs_gridpos = queried_grid[x][y]

            # sort both lists by their uuid
            sent_imgs_gridpos = sorted(
                sent_imgs_gridpos, key=lambda i: i.header.uuid_msgs
            )
            query_imgs_gridpos = sorted(
                query_imgs_gridpos, key=lambda i: i.header.uuid_msgs
            )
            assert sent_imgs_gridpos == query_imgs_gridpos
