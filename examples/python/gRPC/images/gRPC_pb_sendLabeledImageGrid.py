#!/usr/bin/env python3

import math
import time
import uuid
from typing import List, Tuple, Union

import flatbuffers
import numpy as np
from google.protobuf import empty_pb2
from grpc import Channel
from seerep.fb import camera_intrinsics_service_grpc_fb as ci_service
from seerep.pb import camera_intrinsics_pb2 as cameraintrinsics
from seerep.pb import (
    camera_intrinsics_service_pb2_grpc as camintrinsics_service,
)
from seerep.pb import image_pb2 as image
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_category_pb2, label_pb2
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
    target_proj_uuid: Union[str, None] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> Tuple[
    List[List[image.Image]],
    List[Tuple[int, int]],
    cameraintrinsics.CameraIntrinsics,
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
    # A valid camera intrinsics UUID is needed here for succesful storage
    # of Images
    # Add new Camera Intrinsics

    ciuuid = str(uuid.uuid4())

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

    camin.distortion_model = "plumb_bob"

    camin.distortion.extend(list(range(0, 3)))

    camin.intrinsic_matrix.extend([1, 0, 0, 0, 2, 0, 0, 0, 1])
    camin.rectification_matrix.extend(list(range(12, 12 + 9)))
    camin.projection_matrix.extend(list(range(21, 21 + 12)))

    camin.binning_x = 6
    camin.binning_y = 7

    # this deactivates the influence of the frustum matrix on the coordinates
    # of the images, for easier testing
    camin.maximum_viewing_distance = 0

    # old value:
    # camin.maximum_viewing_distance = 5

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
        # create an image more per cell
        # (1 image in first cell; 9 images in 9th cell...)
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

            labelStr = ["label1", "label2"]
            for iCategory in ["category A", "category B"]:
                labelsCategory = label_category_pb2.LabelCategory()
                labelsCategory.category = iCategory
                labelsCategory.datumaroJson = "a very valid datumaro json"

                for labelAct in labelStr:
                    label = label_pb2.Label()
                    label.label = labelAct
                    label.labelIdDatumaro = 1
                    label.instanceUuid = str(uuid.uuid4())
                    label.instanceIdDatumaro = 22
                    labelsCategory.labels.append(label)

                theImage.labels.append(labelsCategory)

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
        target_proj_uuid = getProject(
            builder, grpc_channel, "LabeledImagesInGrid"
        )

    ciuuid = str(uuid.uuid4())

    # 3. Get gRPC service object
    stub = ci_service.CameraIntrinsicsServiceStub(grpc_channel)

    # Create all necessary objects for the query
    ts = createTimeStamp(builder, 4, 3)
    header = createHeader(builder, ts, "map", target_proj_uuid, ciuuid)
    roi = createRegionOfInterest(builder, 3, 5, 6, 7, True)

    matrix = [4, 5, 6, 7, 0.1]
    ci = createCameraIntrinsics(
        builder,
        header,
        3,
        4,
        "plump_bob",
        matrix,
        matrix,
        matrix,
        matrix,
        4,
        5,
        roi,
        5,
    )
    builder.Finish(ci)

    buf = builder.Output()

    stub.TransferCameraIntrinsics(bytes(buf))

    # Fetch the saved CI
    builder = flatbuffers.Builder(1000)

    ci_query = createCameraIntrinsicsQuery(builder, ciuuid, target_proj_uuid)

    builder.Finish(ci_query)
    buf = builder.Output()

    stub.GetCameraIntrinsics(bytes(buf))

    return ciuuid


if __name__ == "__main__":
    grid_list, _, _ = send_labeled_image_grid()
    # seperate the functions print statements from the result
    print()

    for x in range(len(grid_list)):
        for y in range(len(grid_list[x])):
            print(
                "#############################################################"
            )
            print(f"Grid cell {x}, {y} has {len(grid_list[x][y])} images.")
            for img in grid_list[x][y]:
                print(
                    "----------------------------------------------------------"
                )
                print(f"Image {img[0]} has {len(img[1].labels)} labels.")
                for labelCategory in img[1].labels:
                    print(f"\nlabel category: {labelCategory.category}")
                    for label in labelCategory.labels:
                        print(f"label: {label.label}")
                        print(f"labelIdDatumaro: {label.labelIdDatumaro}")
                        print(f"instanceUuid: {label.instanceUuid}")
                        print(f"instanceIdDatumaro: {label.instanceIdDatumaro}")
