#!/usr/bin/env python3

import time
import uuid

import flatbuffers
import numpy as np
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


def add_camintrins(target_proj_uuid: str, grpc_channel) -> str:

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
    ci = createCameraIntrinsics(builder, header, 3, 4, "plump_bob", matrix, matrix, matrix, matrix, 4, 5, roi, 5)
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
    channel = get_gRPC_channel()

    stub = imageService.ImageServiceStub(channel)
    stubTf = tfService.TfServiceStub(channel)
    stubMeta = metaOperations.MetaOperationsStub(channel)

    # create new project
    creation = projectCreation.ProjectCreation(name="LabeledImagesInGrid", mapFrameId="map")
    projectCreated = stubMeta.CreateProject(creation)
    projectname = projectCreated.uuid

    # get and save the time
    theTime = int(time.time())

    stubCI = camintrinsics_service.CameraIntrinsicsServiceStub(channel)
    #####
    # A valid camera intrinsics UUID is needed here for succesful storage of Images
    # Add new Camera Intrinsics

    ciuuid = add_camintrins(projectname, channel)

    # create 9 grid cells
    for k in range(9):
        n = 0
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
            theImage.header.uuid_project = projectname
            theImage.height = lim
            theImage.width = lim
            theImage.encoding = "rgb8"
            theImage.step = 3 * lim
            theImage.data = bytes(rgb)
            theImage.uuid_camera_intrinsics = ciuuid

            for iCategory in range(0, 2):
                bbCat = boundingbox2d_labeled_with_category.BoundingBox2DLabeledWithCategory()
                bbCat.category = str(iCategory)
                # 5. Create bounding boxes with la  bels
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
                labelsCat = labels_with_instance_with_category.LabelsWithInstanceWithCategory()
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

            print("uuid of transfered img: " + uuidImg.message)

    # create tf with data valid for all following tfs
    theTf = tf.TransformStamped()
    theTf.header.frame_id = "map"
    theTf.header.uuid_project = projectname
    theTf.child_frame_id = "camera"
    theTf.transform.translation.z = 0
    theTf.transform.rotation.x = 0
    theTf.transform.rotation.y = 0
    theTf.transform.rotation.z = 0
    theTf.transform.rotation.w = 1

    # create tf for a 3x3 grid
    # x,y -> [0,2]
    for x in range(3):
        for y in range(3):
            # increment time per cell
            theTf.header.stamp.seconds = theTime + (x * 3 + y)
            # write x and y coordinates
            theTf.transform.translation.x = x
            theTf.transform.translation.y = y
            # transfer tf
            stubTf.TransferTransformStamped(theTf)
