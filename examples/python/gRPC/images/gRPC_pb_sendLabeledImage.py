#!/usr/bin/env python3

import os
import sys
import time
import uuid

import numpy as np
from google.protobuf import empty_pb2
from seerep.pb import boundingbox2d_labeled_pb2 as boundingbox2d_labeled
from seerep.pb import (
    boundingbox2d_labeled_with_category_pb2 as boundingbox2d_labeled_with_category,
)
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

# importing util functions. Assuming that this file is in the parent dir
# https://github.com/agri-gaia/seerep/blob/6c4da5736d4a893228e97b01a9ada18620b1a83f/examples/python/gRPC/util.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

# Default server is localhost !
channel = util.get_gRPC_channel(target="local")

# 1. Get gRPC service objects
stub = imageService.ImageServiceStub(channel)
stubTf = tfService.TfServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

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

# A valid camera intrinsics UUID is needed here for succesful storage of Images
# Run the gRPC_pb_addCameraIntrinsics.py script to add a new camera intrinsics
# and paste the id below
camintrinsics_uuid = "c7efb9b3-a952-465a-8038-2d9c9dc39c29"

# 4. Create ten images
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
    theImage.uuid_camera_intrinsics = camintrinsics_uuid

    # Add image data
    theImage.data = bytes(rgb)

    for iCategory in range(0, 2):
        bbCat = boundingbox2d_labeled_with_category.BoundingBox2DLabeledWithCategory()
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
        labelsCat = labels_with_instance_with_category.LabelsWithInstanceWithCategory()
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
