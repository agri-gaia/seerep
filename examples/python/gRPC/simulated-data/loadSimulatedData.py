#!/usr/bin/env python3

import os
import sys
import uuid

import bounding_box_2d_labeled_pb2
import bounding_box_2d_labeled_with_category_pb2
import image_pb2
import image_service_pb2_grpc
import imageio.v2 as imageio
import label_with_instance_pb2
import labels_with_instance_with_category_pb2
import meta_operations_pb2_grpc
import numpy as np
import point_cloud_2_pb2
import point_cloud_service_pb2_grpc
import point_field_pb2
import project_creation_pb2
import tf_service_pb2_grpc
import transform_stamped_pb2
import yaml

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stubImage = image_service_pb2_grpc.ImageServiceStub(channel)
stubPointcloud = point_cloud_service_pb2_grpc.PointCloudServiceStub(channel)
stubTf = tf_service_pb2_grpc.TfServiceStub(channel)
stubMeta = meta_operations_pb2_grpc.MetaOperationsStub(channel)

# create new project
creation = project_creation_pb2.ProjectCreation(name="simulatedData", map_frame_id="map")
projectCreated = stubMeta.CreateProject(creation)
projectname = projectCreated.uuid

# get and save the time
theTime = [1654688921, 1654713534]  # 08.06.2022 13:49, 08.06.2022 20:39 #int(time.time())


root = ["/seerep/seerep-data/simulatedData/lighting_01/", "/seerep/seerep-data/simulatedData/lighting_02/"]
# seerep-data/simulatedData/Beleuchtung_02/2022_06_07_16_52_48/

labelSwitch = {
    1.0: 'http://aims.fao.org/aos/agrovoc/c_14385',
    2.0: 'http://aims.fao.org/aos/agrovoc/c_820',
    3.0: 'http://aims.fao.org/aos/agrovoc/c_7910',
    4.0: 'http://aims.fao.org/aos/agrovoc/c_cb9aeea7',
    5.0: 'http://aims.fao.org/aos/agrovoc/c_71145989',
}

for folderIndex in range(2):
    imagePath = root[folderIndex] + "camera_main_camera/rect/"
    annotationPath = root[folderIndex] + "camera_main_camera_annotations/bounding_box/"
    pointcloudPath = root[folderIndex] + "camera_main_camera_annotations/pointcloud/"

    for i in range(16):
        timeThisIteration = theTime[folderIndex] + i

        baseFilePath = imagePath + str(i).zfill(4)
        baseAnnotationPath = annotationPath + str(i).zfill(4)
        basePointcloudPath = pointcloudPath + str(i).zfill(4)
        im = imageio.imread(baseFilePath + ".png")

        theImage = image_pb2.Image()
        # write the header
        theImage.header.frame_id = "camera"
        theImage.header.stamp.seconds = timeThisIteration
        theImage.header.stamp.nanos = 0
        theImage.header.uuid_project = projectname
        theImage.height = im.shape[0]
        theImage.width = im.shape[1]
        theImage.encoding = "rgb8"
        theImage.step = 3 * im.shape[1]
        theImage.data = im.tobytes()

        annotations = np.genfromtxt(baseAnnotationPath + ".txt", delimiter=" ")

        # write labeled bounding boxes
        bbCat = bounding_box_2d_labeled_with_category_pb2.BoundingBox2DLabeledWithCategory()
        bbCat.category = "simLabels"
        for a in annotations:
            bb = bounding_box_2d_labeled_pb2.BoundingBox2DLabeled()
            bb.label_with_instance.label = labelSwitch.get(a[0])
            bb.label_with_instance.instance_uuid = str(uuid.uuid4())

            bb.bounding_box.point_min.x = a[1] - a[3] / 2.0
            bb.bounding_box.point_min.y = a[2] - a[4] / 2.0
            bb.bounding_box.point_max.x = a[1] + a[3] / 2.0
            bb.bounding_box.point_max.y = a[2] + a[4] / 2.0
            bbCat.labeled_2d_bounding_boxes.append(bb)
        theImage.labeled_bounding_boxes.append(bbCat)

        stubImage.TransferImage(theImage)

        # seerep currently assumes that point cloud coordinates are 32 Bit floats
        pointcloudData = np.load(basePointcloudPath + ".npy").astype(np.float32)

        thePointcloud = point_cloud_2_pb2.PointCloud2()
        thePointcloud.header.frame_id = "camera"
        thePointcloud.header.stamp.seconds = timeThisIteration
        thePointcloud.header.stamp.nanos = 0
        thePointcloud.header.uuid_project = projectname
        thePointcloud.height = pointcloudData.shape[0]
        thePointcloud.width = pointcloudData.shape[1]
        thePointcloud.is_bigendian = False
        thePointcloud.point_step = 12
        thePointcloud.row_step = thePointcloud.width * thePointcloud.point_step
        thePointcloud.data = pointcloudData.tobytes()

        thePointfieldX = point_field_pb2.PointField()
        thePointfieldX.name = 'x'
        thePointfieldX.offset = 0
        thePointfieldX.datatype = 7  # float32
        thePointfieldX.count = 1
        thePointcloud.fields.append(thePointfieldX)

        thePointfieldY = point_field_pb2.PointField()
        thePointfieldY.name = 'y'
        thePointfieldY.offset = 4
        thePointfieldY.datatype = 7  # float32
        thePointfieldY.count = 1
        thePointcloud.fields.append(thePointfieldY)

        thePointfieldZ = point_field_pb2.PointField()
        thePointfieldZ.name = 'z'
        thePointfieldZ.offset = 8
        thePointfieldZ.datatype = 7  # float32
        thePointfieldZ.count = 1
        thePointcloud.fields.append(thePointfieldZ)

        stubPointcloud.TransferPointCloud2(thePointcloud)

        annotations = np.genfromtxt(baseAnnotationPath + ".txt", delimiter=" ")

        labels = labels_with_instance_with_category_pb2.LabelsWithInstanceWithCategory()
        labels.category = "simLabels"
        for a in annotations:
            label = label_with_instance_pb2.LabelWithInstance()
            label.label = labelSwitch.get(a[0])
            labels.label_with_instances.append(label)
        thePointcloud.general_labels.append(labels)

        with open(baseFilePath + ".txt", "r") as stream:
            try:
                pose = yaml.safe_load(stream)
                position = np.fromstring(pose.get('camera_pose_translation'), dtype=float, sep=' ')
                rotation = np.fromstring(pose.get('camera_pose_rotation_quaternion'), dtype=float, sep=' ')

                # create tf with data valid for all following tfs
                theTf = transform_stamped_pb2.TransformStamped()
                theTf.header.frame_id = "map"
                theTf.header.stamp.seconds = timeThisIteration
                theTf.header.uuid_project = projectname
                theTf.child_frame_id = "camera"
                theTf.transform.translation.x = position[0]
                theTf.transform.translation.y = position[1]
                theTf.transform.translation.z = position[2]
                theTf.transform.rotation.x = rotation[0]
                theTf.transform.rotation.y = rotation[1]
                theTf.transform.rotation.z = rotation[2]
                theTf.transform.rotation.w = rotation[3]
                stubTf.TransferTransformStamped(theTf)
            except yaml.YAMLError as exc:
                print(exc)
