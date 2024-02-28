#!/usr/bin/env python3
import sys
from typing import List, Optional

import flatbuffers
from grpc import Channel
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createLabelWithCategory,
    createLabelWithConfidence,
    createPoint2d,
    createPolygon2D,
    createQuery,
    createTimeInterval,
    createTimeStamp,
    getProject,
)


def query_images(
    target_proj_uuid: Optional[str] = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[Image.Image]:

    builder = flatbuffers.Builder(1024)
    if target_proj_uuid is None:
        # 1. Get all projects from the server
        target_proj_uuid = getProject(builder, grpc_channel, "testproject")
        # 2. Check if the defined project exist, else return None
        if target_proj_uuid is None:
            print("valid project doesn't exist! Please execute gRPC_fb_addCameraIntrinsics.py beforehand.")
            return None

    # 3. Get gRPC service object
    stub = imageService.ImageServiceStub(grpc_channel)

    # Create all necessary objects for the query
    l = 100
    polygon_vertices = []
    polygon_vertices.append(createPoint2d(builder, -1.0 * l, -1.0 * l))
    polygon_vertices.append(createPoint2d(builder, -1.0 * l, l))
    polygon_vertices.append(createPoint2d(builder, l, l))
    polygon_vertices.append(createPoint2d(builder, l, -1.0 * l))
    polygon2d = createPolygon2D(builder, 700, -100, polygon_vertices)

    timeMin = createTimeStamp(builder, 1610549273, 0)
    timeMax = createTimeStamp(builder, 1938549273, 0)
    timeInterval = createTimeInterval(builder, timeMin, timeMax)

    projectUuids = [builder.CreateString(target_proj_uuid)]

    # list of categories
    category = ["0"]
    # list of labels per category
    labels = [
        [
            createLabelWithConfidence(builder, "testlabel0"),
            createLabelWithConfidence(builder, "testlabelgeneral0"),
        ]
    ]
    labelCategory = createLabelWithCategory(builder, category, labels)
    dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
    instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

    # 4. Create a query with parameters
    # all parameters are optional
    # with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
    query = createQuery(
        builder,
        polygon2d=polygon2d,
        # timeInterval=timeInterval,
        # labels=labelCategory,
        # mustHaveAllLabels=True,
        projectUuids=projectUuids,
        # instanceUuids=instanceUuids,
        # dataUuids=dataUuids,
        withoutData=True,
        fullyEncapsulated=False,
        inMapFrame=True,
    )
    builder.Finish(query)
    buf = builder.Output()

    # save the queried images in a list
    queried_images = []

    # 5. Query the server for images matching the query and iterate over them
    for responseBuf in stub.GetImage(bytes(buf)):
        response = Image.Image.GetRootAs(responseBuf)
        queried_images.append(response)

    return queried_images


if __name__ == "__main__":
    queried_images = query_images()
    if queried_images is None:
        print("no images were queried")
        sys.exit()
    print(f"count of images: {len(queried_images)}")

    for img in queried_images:
        print("--------------------------------------------------------------------------------------------")
        print(f"uuidmsg: {img.Header().UuidMsgs().decode('utf-8')}")
        print(f"count of bounding box labels: {img.LabelsBbLength()}")
        if img.LabelsBbLength() > 0:
            print(
                "first label: "
                + img.LabelsBb(0).BoundingBox2dLabeled(0).LabelWithInstance().Label().Label().decode("utf-8")
                + " ; confidence: "
                + str(img.LabelsBb(0).BoundingBox2dLabeled(0).LabelWithInstance().Label().Confidence())
            )
            print(
                "first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): "
                + str(img.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().CenterPoint().X())
                + " "
                + str(img.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().CenterPoint().Y())
                + " "
                + str(img.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().SpatialExtent().X())
                + " "
                + str(img.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().SpatialExtent().Y())
                + " "
                + str(img.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().Rotation())
            )
    print("--------------------------------------------------------------------------------------------")
