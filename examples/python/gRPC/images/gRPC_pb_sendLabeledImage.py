#!/usr/bin/env python3

import os
import sys
import time
import uuid

import bounding_box_2d_labeled_pb2
import bounding_box_2d_labeled_with_category_pb2
import image_pb2
import image_service_pb2_grpc
import label_with_instance_pb2
import labels_with_instance_with_category_pb2
import meta_operations_pb2_grpc
import numpy as np
import project_creation_pb2
import tf_service_pb2_grpc
import transform_stamped_pb2
from google.protobuf import empty_pb2

# importing util functions. Assuming that this file is in the parent dir
# https://github.com/agri-gaia/seerep/blob/6c4da5736d4a893228e97b01a9ada18620b1a83f/examples/python/gRPC/util.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

# Default server is localhost !
channel = util.get_gRPC_channel(target="local")

# 1. Get gRPC service objects
stub = image_service_pb2_grpc.ImageServiceStub(channel)
stubTf = tf_service_pb2_grpc.TfServiceStub(channel)
stubMeta = meta_operations_pb2_grpc.MetaOperationsStub(channel)

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
    creation = project_creation_pb2.ProjectCreation(name="testproject", map_frame_id="map")
    projectCreated = stubMeta.CreateProject(creation)
    projectuuid = projectCreated.uuid


theTime = int(time.time())

# 4. Create ten images
for n in range(10):
    theImage = image_pb2.Image()

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
    # Add image data
    theImage.data = bytes(rgb)

    for iCategory in range(0, 2):
        bbCat = bounding_box_2d_labeled_with_category_pb2.BoundingBox2DLabeledWithCategory()
        bbCat.category = str(iCategory)
        # 5. Create bounding boxes with la  bels
        bb = bounding_box_2d_labeled_pb2.BoundingBox2DLabeled()
        for i in range(0, 2):
            bb.label_with_instance.label = "testlabel" + str(i)
            bb.label_with_instance.instance_uuid = str(uuid.uuid4())
            bb.bounding_box.point_min.x = 0.01 + i / 10
            bb.bounding_box.point_min.y = 0.02 + i / 10
            bb.bounding_box.point_max.x = 0.03 + i / 10
            bb.bounding_box.point_max.y = 0.04 + i / 10
            bbCat.labeled_2d_bounding_boxes.append(bb)
        theImage.labeled_bounding_boxes.append(bbCat)

        # # 6. Add general labels to the image
        labelsCat = labels_with_instance_with_category_pb2.LabelsWithInstanceWithCategory()
        labelsCat.category = str(iCategory)
        for i in range(0, 2):
            label = label_with_instance_pb2.LabelWithInstance()
            label.label = "testlabelgeneral" + str(i)
            # assuming that that the general labels are not instance related -> no instance uuid
            # label.instanceUuid = str(uuid.uuid4())
            labelsCat.label_with_instances.append(label)
        theImage.general_labels.append(labelsCat)

    # 7. Send image to the server
    uuidImg = stub.TransferImage(theImage)

    print("uuid of transfered img: " + uuidImg.message)

# 8. Add coordinate transformations and send them to the server
theTf = transform_stamped_pb2.TransformStamped()
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
