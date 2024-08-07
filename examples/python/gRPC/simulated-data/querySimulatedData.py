#!/usr/bin/env python3

import sys

import flatbuffers
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createLabelsWithCategoryVector,
    createPoint2d,
    createPolygon2D,
    createQuery,
    createTimeInterval,
    createTimeStamp,
    getProject,
)

builder = flatbuffers.Builder(1024)
channel = get_gRPC_channel()

# 1. Get all projects from the server
projectuuid = getProject(builder, channel, "simulatedDataWithInstances")

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    sys.exit()

# 3. Get gRPC service object
stub = imageService.ImageServiceStub(channel)

# Create all necessary objects for the query
scale = 5
vertices = [
    createPoint2d(builder, x * scale, y * scale)
    for x, y in [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)]
]
polygon2d = createPolygon2D(builder, 200, -100, vertices)

timeMin = createTimeStamp(builder, 1654688920, 0)
timeMax = createTimeStamp(builder, 1654688940, 0)
timeInterval = createTimeInterval(builder, timeMin, timeMax)

projectUuids = [projectuuid]
labels = createLabelsWithCategoryVector(
    builder, ["ground_truth"], [["Gänseblümchen"]]
)

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set)
# the result of the query will be empty.
# Set the query parameters to adequate values or remove them from
# the query creation
query = createQuery(
    builder,
    # boundingBox=boundingboxStamped,
    # timeInterval=timeInterval,
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
        print("category: " + response.LabelsBb(0).Category().decode("utf-8"))
        print(
            "first label: "
            + response.LabelsBb(0)
            .BoundingBox2dLabeled(0)
            .LabelWithInstance()
            .Label()
            .decode("utf-8")
        )
        print(
            "first BoundingBox (Xmin,Ymin,Xmax,Ymax): "
            + str(
                response.LabelsBb(0)
                .BoundingBox2dLabeled(0)
                .BoundingBox()
                .PointMin()
                .X()
            )
            + " "
            + str(
                response.LabelsBb(0)
                .BoundingBox2dLabeled(0)
                .BoundingBox()
                .PointMin()
                .Y()
            )
            + " "
            + str(
                response.LabelsBb(0)
                .BoundingBox2dLabeled(0)
                .BoundingBox()
                .PointMax()
                .X()
            )
            + " "
            + str(
                response.LabelsBb(0)
                .BoundingBox2dLabeled(0)
                .BoundingBox()
                .PointMax()
                .Y()
            )
        )
