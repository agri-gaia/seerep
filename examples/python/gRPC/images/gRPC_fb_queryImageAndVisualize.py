#!/usr/bin/env python3

t1 = True
gli_image = True

import os
import sys

import flatbuffers
import numpy as np
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
projectuuid = util_fb.getProject(builder, channel, 'plantmap01')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    print("project doesn't exist!")
    exit()

# 3. Get gRPC service object
stub = imageService.ImageServiceStub(channel)


# Create all necessary objects for the query
header = util_fb.createHeader(builder, frame="map")
pointMin = util_fb.createPoint(builder, 2.0, -1.0, -1.0)
pointMax = util_fb.createPoint(builder, 2.1, 0.5, 1.0)
boundingboxStamped = util_fb.createBoundingBoxStamped(builder, header, pointMin, pointMax)

if t1:
    timeMin = util_fb.createTimeStamp(builder, 1661336503, 0)
    timeMax = util_fb.createTimeStamp(builder, 1662336503, 0)
else:
    timeMin = util_fb.createTimeStamp(builder, 1663003785, 0)
    timeMax = util_fb.createTimeStamp(builder, 1664003785, 0)

timeInterval = util_fb.createTimeInterval(builder, timeMin, timeMax)


projectUuids = [builder.CreateString(projectuuid)]
category = ["image_type"]  # , "crops"]
if gli_image:
    imageType = [builder.CreateString("image_type_gli")]
else:
    imageType = [builder.CreateString("image_type_rgb")]
crops = [builder.CreateString("white_cabbage_young")]
labels = [imageType]  # , crops]
labelCategory = util_fb.createLabelWithCategory(builder, category, labels)
dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = util_fb.createQuery(
    builder,
    boundingBox=boundingboxStamped,
    timeInterval=timeInterval,
    labels=labelCategory,
    mustHaveAllLabels=True,
    projectUuids=projectUuids,
    # instanceUuids=instanceUuids,
    # dataUuids=dataUuids,
    withoutData=False,
)
builder.Finish(query)
buf = builder.Output()

# 5. Query the server for images matching the query and iterate over them
counter = 0
for responseBuf in stub.GetImage(bytes(buf)):
    counter = counter + 1
    response = Image.Image.GetRootAs(responseBuf)

    print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
    for i in range(response.LabelsGeneralLength()):
        print(f"\tcategory: {response.LabelsGeneral(i).Category().decode('utf-8')}")
        for j in range(response.LabelsGeneral(i).LabelsWithInstanceLength()):
            print(f"\t\t#{j} label: {response.LabelsGeneral(i).LabelsWithInstance(j).Label().decode('utf-8')}")

    for i in range(response.LabelsBbLength()):
        print(f"\tcategory: {response.LabelsBb(i).Category().decode('utf-8')}")
        for j in range(response.LabelsBb(i).BoundingBox2dLabeledLength()):
            print(
                f"\t\t#{j} label: {response.LabelsBb(i).BoundingBox2dLabeled(j).LabelWithInstance().Label().decode('utf-8'):30}"
                + f"bounding box (Xmin,Ymin,Xmax,Ymax): "
                + str(response.LabelsBb(i).BoundingBox2dLabeled(j).BoundingBox().PointMin().X())
                + " "
                + str(response.LabelsBb(i).BoundingBox2dLabeled(j).BoundingBox().PointMin().Y())
                + " "
                + str(response.LabelsBb(i).BoundingBox2dLabeled(j).BoundingBox().PointMax().X())
                + " "
                + str(response.LabelsBb(i).BoundingBox2dLabeled(j).BoundingBox().PointMax().Y())
            )
    if not response.DataIsNone():
        image = response.DataAsNumpy()
        if response.Encoding().decode('utf-8') == "mono8":
            image.resize(response.Height(), response.Width(), 1)
            image = (image - 100) * 4
        elif response.Encoding().decode('utf-8') == "rgb8":
            image.resize(response.Height(), response.Width(), 3)

        bb = []
        class_ids = []
        if (
            not response.LabelsBbIsNone()
            and response.LabelsBbLength() > 0
            and not response.LabelsBb(0).BoundingBox2dLabeledIsNone()
            and response.LabelsBb(0).BoundingBox2dLabeledLength() > 0
        ):
            for i in range(response.LabelsBb(0).BoundingBox2dLabeledLength()):
                bb.append(
                    [
                        response.LabelsBb(0).BoundingBox2dLabeled(i).BoundingBox().PointMin().Y(),
                        response.LabelsBb(0).BoundingBox2dLabeled(i).BoundingBox().PointMin().X(),
                        response.LabelsBb(0).BoundingBox2dLabeled(i).BoundingBox().PointMax().Y(),
                        response.LabelsBb(0).BoundingBox2dLabeled(i).BoundingBox().PointMax().X(),
                    ]
                )
                if (
                    response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().Label().decode('utf-8')
                    == "white_cabbage_young"
                ):
                    class_ids.append(0)
                elif (
                    response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().Label().decode('utf-8')
                    == "white_cabbage"
                ):
                    class_ids.append(1)
                elif (
                    response.LabelsBb(0).BoundingBox2dLabeled(i).LabelWithInstance().Label().decode('utf-8')
                    == "white_cabbage_harvested"
                ):
                    class_ids.append(2)

        classnames = ["white cabbage young", "white cabbage", "white cabbage harvested"]

        util.display_instances(
            image,
            np.array(bb),
            np.array(class_ids),
            classnames,
            image_name=response.Header().UuidMsgs().decode('utf-8') + ".png",
            gli_image=gli_image,
            rotate=t1,
        )

print(f"Received {counter} images")
