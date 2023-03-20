#!/usr/bin/env python3

import os
import sys

from google.protobuf import empty_pb2
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_pb2
from seerep.pb import labels_with_category_pb2 as labels_with_category
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import query_pb2 as query

# importing util functions. Assuming that this file is in the parent dir
# https://github.com/agri-gaia/seerep/blob/6c4da5736d4a893228e97b01a9ada18620b1a83f/examples/python/gRPC/util.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

# Default server is localhost !
channel = util.get_gRPC_channel()

# 1. Get gRPC service objects
stub = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

# 2. Get all projects from the server
response = stubMeta.GetProjects(empty_pb2.Empty())

# 3. Check if we have an existing test project, if not, we stop here
projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid + "\n")
    if project.name == "testproject":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()

# 4. Create a query with parameters
theQuery = query.Query()
theQuery.projectuuid.append(projectuuid)
theQuery.boundingboxStamped.header.frame_id = "map"

theQuery.boundingboxStamped.boundingbox.center_point.x = 50.0
theQuery.boundingboxStamped.boundingbox.center_point.y = 50.0
theQuery.boundingboxStamped.boundingbox.center_point.z = 50.0
theQuery.boundingboxStamped.boundingbox.spatial_extent.x = 100.0
theQuery.boundingboxStamped.boundingbox.spatial_extent.y = 100.0
theQuery.boundingboxStamped.boundingbox.spatial_extent.z = 100.0

# since epoche
theQuery.timeinterval.time_min.seconds = 1638549273
theQuery.timeinterval.time_min.nanos = 0
theQuery.timeinterval.time_max.seconds = 1938549273
theQuery.timeinterval.time_max.nanos = 0

# labels
label = labels_with_category.LabelsWithCategory()
label.category = "0"
labelWithConfidence = label_pb2.Label()
labelWithConfidence.label = "testlabel0"
label.labels.extend([labelWithConfidence])
theQuery.labelsWithCategory.append(label)

# 5. Query the server for images matching the query and iterate over them
for img in stub.GetImage(theQuery):
    print(
        f"uuidmsg: {img.header.uuid_msgs}"
        + "\n"
        + f"first label: {img.labels_bb[0].boundingBox2DLabeled[0].labelWithInstance.label.label}"
        + "\n"
        + f"first label confidence: {img.labels_bb[0].boundingBox2DLabeled[0].labelWithInstance.label.confidence}"
        + "\n"
        + "First bounding box (Xcenter,Ycenter,Xextent,Yextent):"
        + " "
        + str(img.labels_bb[0].boundingBox2DLabeled[0].boundingBox.center_point.x)
        + " "
        + str(img.labels_bb[0].boundingBox2DLabeled[0].boundingBox.center_point.y)
        + " "
        + str(img.labels_bb[0].boundingBox2DLabeled[0].boundingBox.spatial_extent.x)
        + " "
        + str(img.labels_bb[0].boundingBox2DLabeled[0].boundingBox.spatial_extent.y)
        + "\n"
    )
