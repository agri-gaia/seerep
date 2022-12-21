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

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel("local")

stub = image_service_pb2_grpc.ImageServiceStub(channel)
stubTf = tf_service_pb2_grpc.TfServiceStub(channel)
stubMeta = meta_operations_pb2_grpc.MetaOperationsStub(channel)

# create new project
creation = project_creation_pb2.ProjectCreation(name="LabeledImagesInGrid", map_frame_id="map")
projectCreated = stubMeta.CreateProject(creation)
projectname = projectCreated.uuid

# get and save the time
theTime = int(time.time())

# create 9 grid cells
for k in range(9):
    n = 0
    # create an image more per cell (1 image in first cell; 9 images in 9th cell...)
    while n <= k:
        n = n + 1
        theImage = image_pb2.Image()

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

        # transfer image
        uuidImg = stub.TransferImage(theImage)

        print("uuid of transfered img: " + uuidImg.message)

# create tf with data valid for all following tfs
theTf = transform_stamped_pb2.TransformStamped()
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
