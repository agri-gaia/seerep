#!/usr/bin/env python3

import math
import time
import uuid
from typing import List, Tuple

import flatbuffers
import numpy as np
from google.protobuf import empty_pb2
from grpc import Channel
from seerep.fb import CameraIntrinsics
from seerep.fb import camera_intrinsics_service_grpc_fb as ci_service
from seerep.pb import boundingbox2d_labeled_pb2 as boundingbox2d_labeled
from seerep.pb import (
    boundingbox2d_labeled_with_category_pb2 as boundingbox2d_labeled_with_category,
)
from seerep.pb import camera_intrinsics_pb2 as cameraintrinsics
from seerep.pb import camera_intrinsics_service_pb2_grpc as camintrinsics_service
from seerep.pb import image_pb2 as image
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_with_instance_pb2 as labelWithInstance
from seerep.pb import (
    labels_with_instance_with_category_pb2 as labels_with_instance_with_category,
)
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2 as projectCreation
from seerep.pb import tf_service_pb2_grpc as tfService
from seerep.pb import transform_stamped_pb2 as tf
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createCameraIntrinsics,
    createCameraIntrinsicsQuery,
    createHeader,
    createRegionOfInterest,
    createTimeStamp,
    getProject,
)


def send_labeled_image_grid(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> Tuple[
    List[List[image.Image]], List[Tuple[int, int]], cameraintrinsics.CameraIntrinsics
]:

    stub = imageService.ImageServiceStub(grpc_channel)
    stubTf = tfService.TfServiceStub(grpc_channel)
    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)
    stubCI = camintrinsics_service.CameraIntrinsicsServiceStub(grpc_channel)

    if target_proj_uuid is None:
        # 2. Get all projects from the server
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            print(project.name + " " + project.uuid)
            if project.name == "LabeledImagesInGrid":
                target_proj_uuid = project.uuid

        if target_proj_uuid is None:
            creation = projectCreation.ProjectCreation(
                name="LabeledImagesInGrid", mapFrameId="map"
            )
            projectCreated = stubMeta.CreateProject(creation)
            target_proj_uuid = projectCreated.uuid

    #####
    # A valid camera intrinsics UUID is needed here for succesful storage of Images
    # Add new Camera Intrinsics

    ciuuid = add_camintrins(target_proj_uuid, grpc_channel)

    camin = cameraintrinsics.CameraIntrinsics()

    camin.header.stamp.seconds = 4
    camin.header.stamp.nanos = 3

    camin.header.frame_id = "camintrinsics"

    camin.header.uuid_project = target_proj_uuid
    camin.header.uuid_msgs = ciuuid

    camin.region_of_interest.x_offset = 2
    camin.region_of_interest.y_offset = 1
    camin.region_of_interest.height = 5
    camin.region_of_interest.width = 4
    camin.region_of_interest.do_rectify = 4

    camin.height = 5
    camin.width = 4

    camin.distortion_model = "plump_bob"

    camin.distortion.extend([3, 4, 5])

    camin.intrinsic_matrix.extend([3, 4, 5])
    camin.rectification_matrix.extend([3, 4, 5])
    camin.projection_matrix.extend([3, 4, 5])

    camin.binning_x = 6
    camin.binning_y = 7

    camin.maximum_viewing_distance = 5

    stubCI.TransferCameraIntrinsics(camin)

    # get and save the time
    theTime = int(time.time())

    grid_imgs: List[List[image.Image]] = []
    idx_oldy = -1

    # create 9 grid cells
    for k in range(9):
        n = 0
        idx_y = math.floor(k / 3)
        idx_x = k % 3

        if idx_oldy != idx_y:
            idx_oldy = idx_y
            grid_imgs.append([])
        grid_imgs[idx_y].append([])
        # create an image more per cell (1 image in first cell; 9 images in 9th cell...)
        while n <= k:
            n = n + 1
            theImage = image.Image()

            # create rgb with shifted rgb values in each cell
            rgb = []
            lim = 256
            for i in range(lim):
                for j in range(lim):
                    x = float(i) / lim
                    y = float(j) / lim
                    z = float(j) / lim
                    r = np.ubyte((x * 255.0 + k * 100) % 255)
                    g = np.ubyte((y * 255.0 + k * 100) % 255)
                    b = np.ubyte((z * 255.0 + k * 100) % 255)
                    # print(r, g, b)
                    rgb.append(r)
                    rgb.append(g)
                    rgb.append(b)

            # write the header
            theImage.header.frame_id = "camera"
            # increment timestamp for each cell for later matching with tf
            theImage.header.stamp.seconds = theTime + k
            theImage.header.stamp.nanos = 0
            theImage.header.uuid_project = target_proj_uuid
            theImage.height = lim
            theImage.width = lim
            theImage.encoding = "rgb8"
            theImage.step = 3 * lim
            theImage.uuid_camera_intrinsics = ciuuid

            # Add image data
            theImage.data = bytes(rgb)

            for iCategory in range(0, 2):
                bbCat = (
                    boundingbox2d_labeled_with_category.BoundingBox2DLabeledWithCategory()
                )
                bbCat.category = str(iCategory)
                # 5. Create bounding boxes with labels
                bb = boundingbox2d_labeled.BoundingBox2DLabeled()
                for i in range(0, 2):
                    bb.labelWithInstance.label.label = "testlabel" + str(i)
                    bb.labelWithInstance.label.confidence = i / 10.0
                    bb.labelWithInstance.instanceUuid = str(uuid.uuid4())
                    bb.boundingBox.center_point.x = 0.01 + i / 10
                    bb.boundingBox.center_point.y = 0.02 + i / 10
                    bb.boundingBox.spatial_extent.x = 0.03 + i / 10
                    bb.boundingBox.spatial_extent.y = 0.04 + i / 10
                    bbCat.boundingBox2DLabeled.append(bb)
                theImage.labels_bb.append(bbCat)

            # # 6. Add general labels to the image
            labelsCat = (
                labels_with_instance_with_category.LabelsWithInstanceWithCategory()
            )
            labelsCat.category = str(iCategory)
            for i in range(0, 2):
                label = labelWithInstance.LabelWithInstance()
                label.label.label = "testlabelgeneral" + str(i)
                label.label.confidence = i / 10.0
                # assuming that that the general labels are not instance related -> no instance uuid
                label.instanceUuid = str(uuid.uuid4())
                labelsCat.labelWithInstance.append(label)
            theImage.labels_general.append(labelsCat)

            # transfer image
            uuidImg = stub.TransferImage(theImage)

            # put the images uuid and the images in the grid list
            grid_imgs[idx_y][idx_x].append((uuidImg.message, theImage))

    # create tf with data valid for all following tfs
    theTf = tf.TransformStamped()
    theTf.header.frame_id = "map"
    theTf.header.uuid_project = target_proj_uuid
    theTf.child_frame_id = "camera"
    theTf.transform.translation.z = 0
    theTf.transform.rotation.x = 0
    theTf.transform.rotation.y = 0
    theTf.transform.rotation.z = 0
    theTf.transform.rotation.w = 1

    tf_times = []

    # create tf for a 3x3 grid
    # x,y -> [0,2]
    for x in range(3):
        for y in range(3):
            timestamp_s = theTime + (x * 3 + y)
            timestamp_nanos = 0

            # increment time per cell
            theTf.header.stamp.seconds = timestamp_s
            theTf.header.stamp.nanos = timestamp_nanos

            # write x and y coordinates
            theTf.transform.translation.x = x
            theTf.transform.translation.y = y
            # transfer tf
            stubTf.TransferTransformStamped(theTf)
            tf_times.append((timestamp_s, timestamp_nanos))

    return grid_imgs, tf_times, camin


def add_camintrins(target_proj_uuid: str, grpc_channel: Channel) -> str:

    builder = flatbuffers.Builder(1000)

    # 1. Get all projects from the server when no target specified
    if target_proj_uuid is None:
        target_proj_uuid = getProject(builder, grpc_channel, "LabeledImagesInGrid")

    ciuuid = str(uuid.uuid4())

    # 3. Get gRPC service object
    stub = ci_service.CameraIntrinsicsServiceStub(grpc_channel)

    # Create all necessary objects for the query
    ts = createTimeStamp(builder, 4, 3)
    header = createHeader(builder, ts, "map", target_proj_uuid, ciuuid)
    roi = createRegionOfInterest(builder, 3, 5, 6, 7, True)

    matrix = [4, 5, 6]
    ci = createCameraIntrinsics(
        builder, header, 3, 4, "plump_bob", matrix, matrix, matrix, matrix, 4, 5, roi, 5
    )
    builder.Finish(ci)

    buf = builder.Output()

    stub.TransferCameraIntrinsics(bytes(buf))

    # Fetch the saved CI
    builder = flatbuffers.Builder(1000)

    ci_query = createCameraIntrinsicsQuery(builder, ciuuid, target_proj_uuid)

    builder.Finish(ci_query)
    buf = builder.Output()

    ret = stub.GetCameraIntrinsics(bytes(buf))

    retrieved_ci = CameraIntrinsics.CameraIntrinsics.GetRootAs(ret)

    return ciuuid


if __name__ == "__main__":
    grid_list, _, _ = send_labeled_image_grid()
    # seperate the functions print statements from the result
    print()

    for x in range(len(grid_list)):
        for y in range(len(grid_list[x])):
            print("#############################################################")
            print(f"Grid cell {x}, {y} has {len(grid_list[x][y])} images.")
            for img in grid_list[x][y]:
                print("-------------------------------------------------------------")
                print(
                    f"Image {img[0]} has {len(img[1].labels_general)} general labels."
                )
                for label in img[1].labels_general:
                    print(f"General label category: {label.category}")
                    for label2d in label.labelWithInstance:
                        print(f"General label label: {label2d.label.label}")
                        print(f"General label instance uuid: {label2d.instanceUuid}")
                # print("-------------------------------------------------------------")
                # print(f"Image {img[0]} has {len(img[1].labels_bb)} bounding boxes.")
                # for bb in img[1].labels_bb:
                #     print(f"Bounding box category: {bb.category}")
                #     for bb2d in bb.boundingBox2DLabeled:
                #         print(f"Bounding box label: {bb2d.labelWithInstance.label.label}")
                #         print(f"Bounding box instance uuid: {bb2d.labelWithInstance.instanceUuid}")
                #         print(f"Bounding box center point: {bb2d.boundingBox.center_point.x}, {bb2d.boundingBox.center_point.y}")
                #         print(f"Bounding box spatial extent: {bb2d.boundingBox.spatial_extent.x}, {bb2d.boundingBox.spatial_extent.y}")
