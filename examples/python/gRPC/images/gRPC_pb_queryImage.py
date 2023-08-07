#!/usr/bin/env python3

import sys
from typing import List

from google.protobuf import empty_pb2
from seerep.pb import image_pb2 as image
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_pb2
from seerep.pb import labels_with_category_pb2 as labels_with_category
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import point2d_pb2 as point2d
from seerep.pb import query_pb2 as query
from seerep.util.common import get_gRPC_channel


def query_images(
    grpc_channel=get_gRPC_channel(), target_project_uuid=None
) -> List[image.Image]:
    # 1. Get gRPC service objects
    stub = imageService.ImageServiceStub(grpc_channel)
    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)

    # 2. Get all projects from the server
    response = stubMeta.GetProjects(empty_pb2.Empty())

    # 3. Check if we have an existing test project, if not, we stop here
    if not target_project_uuid:
        for project in response.projects:
            print(project.name + " " + project.uuid + "\n")
            if project.name == "testproject":
                target_project_uuid = project.uuid

        if not target_project_uuid:
            sys.exit()

    # 4. Create a query with parameters
    theQuery = query.Query()
    theQuery.projectuuid.append(target_project_uuid)

    theQuery.polygon.z = -1
    theQuery.polygon.height = 7

    theQuery.inMapFrame = False

    l = 100
    bottom_left = point2d.Point2D()
    bottom_left.x = -l
    bottom_left.y = -l
    theQuery.polygon.vertices.append(bottom_left)

    top_left = point2d.Point2D()
    top_left.x = -l
    top_left.y = l
    theQuery.polygon.vertices.append(top_left)

    top_right = point2d.Point2D()
    top_right.x = l
    top_right.y = l
    theQuery.polygon.vertices.append(top_right)

    bottom_right = point2d.Point2D()
    bottom_right.x = l
    bottom_right.y = -l
    theQuery.polygon.vertices.append(bottom_right)

    # since epoche
    # theQuery.timeinterval.time_min.seconds = 1638549273
    # theQuery.timeinterval.time_min.nanos = 0
    # theQuery.timeinterval.time_max.seconds = 1938549273
    # theQuery.timeinterval.time_max.nanos = 0

    # labels
    # label = labels_with_category.LabelsWithCategory()
    # label.category = "0"
    # labelWithConfidence = label_pb2.Label()
    # labelWithConfidence.label = "testlabel0"
    # label.labels.extend([labelWithConfidence])
    # theQuery.labelsWithCategory.append(label)

    # theQuery.inmapframe = True
    theQuery.fullyEncapsulated = False

    queried_img_list: List[image.Image] = []

    # 5. Query the server for images matching the query and iterate over them
    for img in stub.GetImage(theQuery):
        queried_img_list.append(img)
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
    return queried_img_list


if __name__ == "__main__":
    query_images()
