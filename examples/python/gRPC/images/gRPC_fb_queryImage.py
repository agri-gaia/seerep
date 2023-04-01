#!/usr/bin/env python3
import os
import sys

import flatbuffers
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService

# importing util functions. Assuming that these files are in the parent dir
# examples/python/gRPC/util.py
# examples/python/gRPC/util_fb.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)
# Default server is localhost !
channel = util.get_gRPC_channel()

# 1. Get all projects from the server
projectuuid = util_fb.getProject(builder, channel, 'simulatedCropsGroundTruth')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    print("project doesn't exist!")
    exit()

# 3. Get gRPC service object
stub = imageService.ImageServiceStub(channel)


# Create all necessary objects for the query
header = util_fb.createHeader(builder, frame="map")
pointMin = util_fb.createPoint(builder, 0.0, 0.0, 0.0)
pointMax = util_fb.createPoint(builder, 100.0, 100.0, 100.0)
boundingboxStamped = util_fb.createBoundingBoxStamped(builder, header, pointMin, pointMax)

timeMin = util_fb.createTimeStamp(builder, 1610549273, 0)
timeMax = util_fb.createTimeStamp(builder, 1938549273, 0)
timeInterval = util_fb.createTimeInterval(builder, timeMin, timeMax)


projectUuids = [builder.CreateString(projectuuid)]
# list of categories
category = ["0"]
# list of labels per category
labels = [
    [
        util_fb.createLabelWithConfidence(builder, "testlabel0"),
        util_fb.createLabelWithConfidence(builder, "testlabelgeneral0"),
    ]
]
labelCategory = util_fb.createLabelWithCategory(builder, category, labels)
dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = util_fb.createQuery(
    builder,
    # boundingBox=boundingboxStamped,
    # timeInterval=timeInterval,
    # labels=labelCategory,
    # mustHaveAllLabels=True,
    projectUuids=projectUuids,
    # instanceUuids=instanceUuids,
    # dataUuids=dataUuids,
    withoutData=True,
)
builder.Finish(query)
buf = builder.Output()

# 5. Query the server for images matching the query and iterate over them
for responseBuf in stub.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)

    print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
    print(response.LabelsBbLength())
    if response.LabelsBbLength() > 0:
        print(
            "first label: "
            + response.LabelsBb(0).BoundingBox2dLabeled(0).LabelWithInstance().Label().Label().decode("utf-8")
            + " ; confidence: "
            + str(response.LabelsBb(0).BoundingBox2dLabeled(0).LabelWithInstance().Label().Confidence())
        )
        print(
            "first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): "
            + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().CenterPoint().X())
            + " "
            + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().CenterPoint().Y())
            + " "
            + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().SpatialExtent().X())
            + " "
            + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().SpatialExtent().Y())
            + " "
            + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().Rotation())
            + "\n"
        )
