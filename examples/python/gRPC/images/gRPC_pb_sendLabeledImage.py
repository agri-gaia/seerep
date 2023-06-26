#!/usr/bin/env python3

import os
import sys
import time
import uuid
from typing import List

import numpy as np
from google.protobuf import empty_pb2
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


def send_labeled_images() -> List[image.Image]:
    import gRPC_fb_addCameraIntrinsics as camintrins_uuid

    # Default server is localhost !
    channel = get_gRPC_channel(target="local")

    # 1. Get gRPC service objects
    stub = imageService.ImageServiceStub(channel)
    stubTf = tfService.TfServiceStub(channel)
    stubMeta = metaOperations.MetaOperationsStub(channel)
    stubCI = camintrinsics_service.CameraIntrinsicsServiceStub(channel)

    # 2. Get all projects from the server
    response = stubMeta.GetProjects(empty_pb2.Empty())

    # 3. Check if we have an existing test project, if not, one is created.
    found = False
    for project in response.projects:
        print(project.name + " " + project.uuid)
        if project.name == "testproject":
            projectuuid = project.uuid
            found = True

    if not found:
        creation = projectCreation.ProjectCreation(name="testproject", mapFrameId="map")
        projectCreated = stubMeta.CreateProject(creation)
        projectuuid = projectCreated.uuid

    theTime = int(time.time())

    #####
    # A valid camera intrinsics UUID is needed here for succesful storage of Images
    # Add new Camera Intrinsics

    ciuuid = camintrins_uuid.get_ciuuid()
    print("Camera Intrinsics will be saved against the uuid: ", ciuuid)

    camin = cameraintrinsics.CameraIntrinsics()

    camin.header.stamp.seconds = 4
    camin.header.stamp.nanos = 3

    camin.header.frame_id = "camintrinsics"

    camin.header.uuid_project = projectuuid
    camin.header.uuid_msgs = ciuuid

    camin.region_of_interest.x_offset = 2
    camin.region_of_interest.y_offset = 1
    camin.region_of_interest.height = 5
    camin.region_of_interest.width = 4
    camin.region_of_interest.do_rectify = 4

    camin.height = 5
    camin.width = 4

    camin.distortion_model = "plumb_bob"

    camin.distortion.extend([3, 4, 5])

    camin.intrinsic_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11])
    camin.rectification_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11])
    camin.projection_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14])

    camin.binning_x = 6
    camin.binning_y = 7

    stubCI.TransferCameraIntrinsics(camin)

    # 4. Create ten images
    # for debugging and testing this example add all sent images to a list
    sent_images_list = []
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
        theImage.header.uuid_project = projectuuid
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
                # label.instanceUuid = str(uuid.uuid4())
                labelsCat.labelWithInstance.append(label)
            theImage.labels_general.append(labelsCat)

        # 7. Send image to the server
        uuidImg = stub.TransferImage(theImage)

        sent_images_list.append(theImage)
        print("uuid of transfered img: " + uuidImg.message)

    # 8. Add coordinate transformations and send them to the server
    theTf = tf.TransformStamped()
    theTf.header.frame_id = "map"
    theTf.header.stamp.seconds = theTime
    theTf.header.uuid_project = projectuuid
    theTf.child_frame_id = "camera"
    theTf.transform.translation.x = 1
    theTf.transform.translation.y = 2
    theTf.transform.translation.z = 3
    theTf.transform.rotation.x = 0
    theTf.transform.rotation.y = 0
    theTf.transform.rotation.z = 0
    theTf.transform.rotation.w = 1
    stubTf.TransferTransformStamped(theTf)

    theTf.header.stamp.seconds = theTime + 10
    theTf.transform.translation.x = 100
    theTf.transform.translation.y = 200
    theTf.transform.translation.z = 300
    stubTf.TransferTransformStamped(theTf)

    # return sent data
    return sent_images_list


if __name__ == "__main__":
    print(send_labeled_images())
