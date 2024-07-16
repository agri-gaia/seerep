#!/usr/bin/env python3
# NOTE: This file is referenced in the following mkdocs files:
#   images.md
# Any changes done in here will be reflected in there
import time
import uuid
from typing import List, Optional, Tuple

import numpy as np
from google.protobuf import empty_pb2
from grpc import Channel
from seerep.pb import camera_intrinsics_pb2 as cameraintrinsics
from seerep.pb import camera_intrinsics_service_pb2_grpc as camintrinsics_service
from seerep.pb import image_pb2 as image
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_category_pb2, label_pb2
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2 as projectCreation
from seerep.pb import tf_service_pb2_grpc as tfService
from seerep.pb import transform_stamped_pb2 as tf
from seerep.util.common import get_gRPC_channel


def send_labeled_images(
    target_proj_uuid: Optional[str] = None, grpc_channel: Channel = get_gRPC_channel()
) -> Tuple[List[List[image.Image]], List[int], cameraintrinsics.CameraIntrinsics]:
    """sends test images via the given grpc_channel to the specified target project uuid"""

    # 1. Get gRPC service objects
    stub = imageService.ImageServiceStub(grpc_channel)
    stubTf = tfService.TfServiceStub(grpc_channel)
    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)
    stubCI = camintrinsics_service.CameraIntrinsicsServiceStub(grpc_channel)

    # 2. Check if we have an existing test project (or target_proj_uuid is set), if not, one is created.
    if target_proj_uuid is None:
        # 3. Get all projects from the server
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            print(project.name + " " + project.uuid)
            if project.name == "testproject":
                target_proj_uuid = project.uuid

        if target_proj_uuid is None:
            # 4. create a project
            creation = projectCreation.ProjectCreation(name="testproject", mapFrameId="map")
            projectCreated = stubMeta.CreateProject(creation)
            target_proj_uuid = projectCreated.uuid

    theTime = int(time.time())

    #####
    # A valid camera intrinsics UUID is needed here for succesful storage of Images
    # Add new Camera Intrinsics with placeholder data

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

    camin.intrinsic_matrix.extend([3, 4, 5, 10, 7, 8, 9, 10, 11])
    camin.rectification_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11])
    camin.projection_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14])

    camin.binning_x = 6
    camin.binning_y = 7

    camin.maximum_viewing_distance = 5

    stubCI.TransferCameraIntrinsics(camin)

    # 5. Create ten images
    # for debugging and testing this example add all sent images to a list
    sent_images_list: List[Tuple[str, image.Image]] = []

    for n in range(10):
        theImage = image.Image()

        rgb = []
        lim = 256  # 256 x 256 pixels
        for i in range(lim):
            for j in range(lim):
                x = float(i) / lim
                y = float(j) / lim
                z = float(j) / lim
                r = np.ubyte((x * 255.0 + n) % 255)
                g = np.ubyte((y * 255.0 + n) % 255)
                b = np.ubyte((z * 255.0 + n) % 255)
                rgb.append(r)
                rgb.append(g)
                rgb.append(b)

        # Add image meta-data
        theImage.header.frame_id = "camera"
        theImage.header.stamp.seconds = theTime + n
        theImage.header.stamp.nanos = 0
        theImage.header.uuid_project = target_proj_uuid
        theImage.height = lim
        theImage.width = lim
        theImage.encoding = "rgb8"
        theImage.step = 3 * lim
        theImage.uuid_camera_intrinsics = ciuuid

        # Add image data
        theImage.data = bytes(rgb)

        # 5. create label
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

        # 8. Send image to the server
        uuidImg = stub.TransferImage(theImage)

        # also add them to the list
        sent_images_list.append((uuidImg.message, theImage))

    # a list for debugging and testing, if interpolated coordinates
    # according to timestamp of the images and tfs are correct
    tf_times: List[int] = []

    # 8. Add coordinate transformations and send them to the server
    theTf = tf.TransformStamped()
    theTf.header.frame_id = "map"
    theTf.header.stamp.seconds = theTime
    theTf.header.uuid_project = target_proj_uuid
    theTf.child_frame_id = "camera"
    theTf.transform.translation.x = 1
    theTf.transform.translation.y = 2
    theTf.transform.translation.z = 3
    theTf.transform.rotation.x = 0
    theTf.transform.rotation.y = 0
    theTf.transform.rotation.z = 0
    theTf.transform.rotation.w = 1
    stubTf.TransferTransformStamped(theTf)
    tf_times.append(theTf.header.stamp.seconds)

    theTf.header.stamp.seconds = theTime + 10
    theTf.transform.translation.x = 100
    theTf.transform.translation.y = 200
    theTf.transform.translation.z = 300
    stubTf.TransferTransformStamped(theTf)
    tf_times.append(theTf.header.stamp.seconds)

    # return sent data
    return sent_images_list, tf_times, camin


if __name__ == "__main__":
    sent_image_ls_data, _, _ = send_labeled_images()
    camera_intrinsics_allimgs = {intrins_uuid[1].uuid_camera_intrinsics for intrins_uuid in sent_image_ls_data}

    # print statement to seperate the messages of the function
    print()
    print(f"camera intrinsics will be saved against the uuid(s): {camera_intrinsics_allimgs}")
    img_uuids = [img[0] for img in sent_image_ls_data]
    for i, img_uuid in enumerate(img_uuids):
        print(f"the uuid of the sent image number {i} is: {img_uuid}")
