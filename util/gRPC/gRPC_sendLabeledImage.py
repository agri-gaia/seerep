#!/usr/bin/env python3

import sys
import time
import numpy as np

import grpc
import transfer_sensor_msgs_pb2_grpc as transferMsgs
import image_pb2 as image
import boundingbox2d_labeled_pb2 as bb
import projectCreation_pb2 as projectCreation

from google.protobuf import empty_pb2

channel = grpc.insecure_channel("localhost:9090")

stub = transferMsgs.TransferSensorMsgsStub(channel)

response = stub.GetProjects(empty_pb2.Empty())
if not response.uuids:
    creation = projectCreation.ProjectCreation(name="testproject")
    projectCreated = stub.CreateProject(creation)
    projectname = projectCreated.uuid
else:
    print(response.uuids)
    projectname = response.uuids[0]


theImage = image.Image()

rgb = []
lim = 256
for i in range(lim):
    for j in range(lim):
        x = float(i) / lim
        y = float(j) / lim
        z = float(j) / lim
        r = np.ubyte((x * 255.0) % 255)
        g = np.ubyte((y * 255.0) % 255)
        b = np.ubyte((z * 255.0) % 255)
        # print(r, g, b)
        rgb.append(r)
        rgb.append(g)
        rgb.append(b)

theImage.header.frame_id = "map"
theImage.header.stamp.seconds = int(time.time())
theImage.header.uuid_project = projectname
theImage.height = lim
theImage.width = lim
theImage.encoding = "rgb8"
theImage.step = 3 * lim
theImage.data = bytes(rgb)

bb1 = bb.BoundingBox2DLabeled()
for i in range(0, 10):
    bb1.label = "testlabel" + str(i)
    bb1.boundingBox.point_min.x = 0.01 + i / 10
    bb1.boundingBox.point_min.y = 0.02 + i / 10
    bb1.boundingBox.point_max.x = 0.03 + i / 10
    bb1.boundingBox.point_max.y = 0.04 + i / 10
    theImage.labels_bb.append(bb1)

for i in range(0, 10):
    theImage.labels_general.append("testlabelgeneral" + str(i))

uuidImg = stub.TransferImage(theImage)

print("uuid of transfered img: " + uuidImg.message)
