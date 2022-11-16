#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import Image
from fb import image_service_grpc_fb as imageService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)
channel = util.get_gRPC_channel()

# 1. Get all projects from the server
projectuuid = util_fb.getProject(builder, channel, 'testproject')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    exit()

# 3. Get gRPC service object
stub = imageService.ImageServiceStub(channel)

# Create all necessary objects for the query
header = util_fb.createHeader(builder, frame="map")
pointMin = util_fb.createPoint(builder, -5.0, -5.0, -100.0)
pointMax = util_fb.createPoint(builder, 5.0, 5.0, 100.0)
boundingboxStamped = util_fb.createBoundingBoxStamped(builder, header, pointMin, pointMax)

timeMin = util_fb.createTimeStamp(builder, 1654688920, 0)
timeMax = util_fb.createTimeStamp(builder, 1654688940, 0)
timeInterval = util_fb.createTimeInterval(builder, timeMin, timeMax)

projectUuids = [builder.CreateString(projectuuid)]
labels = [builder.CreateString("http://aims.fao.org/aos/agrovoc/c_14385")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = util_fb.createQuery(
    builder,
    boundingBox=boundingboxStamped,
    timeInterval=timeInterval,
    labels=labels,
    projectUuids=projectUuids,
    # instanceUuids=instanceUuids,
    # dataUuids=dataUuids,
    withoutData=True,
)
builder.Finish(query)
buf = builder.Output()

i = 0
for responseBuf in stub.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)
    print(str(i) + "uuidmsg: " + response.Header().UuidMsgs().decode("utf-8"))
    i = i + 1
    if response.LabelsBbLength() > 0:
        print("first label: " + response.LabelsBb(0).LabelWithInstance().Label().decode("utf-8"))
        print(
            "first BoundingBox (Xmin,Ymin,Xmax,Ymax): "
            + str(response.LabelsBb(0).BoundingBox().PointMin().X())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMin().Y())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMax().X())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMax().Y())
        )
