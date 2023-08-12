#!/usr/bin/env python3

import sys
from typing import List, Optional

from google.protobuf import empty_pb2
from grpc import Channel
from seerep.pb import image_pb2 as image
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_pb2
from seerep.pb import labels_with_category_pb2 as labels_with_category
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import point2d_pb2 as point2d
from seerep.pb import query_pb2 as query
from seerep.util.common import get_gRPC_channel


def query_image_grid(
    target_project_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> List[List[image.Image]]:
    stub = imageService.ImageServiceStub(grpc_channel)
    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)

    response = stubMeta.GetProjects(empty_pb2.Empty())

    if target_project_uuid is None:
        for project in response.projects:
            print(project.name + " " + project.uuid)
            if project.name == "LabeledImagesInGrid":
                target_project_uuid = project.uuid

        if target_project_uuid is None:
            print(
                "No project with name 'LabeledImagesInGrid' found! Execute gRPC_pb_sendLabeledImageGrid.py beforehand!"
            )
            sys.exit()

    theQuery = query.Query()
    theQuery.projectuuid.append(target_project_uuid)

    l = 1
    t = 4
    bottom_left = point2d.Point2D()
    bottom_left.x = -l + t
    bottom_left.y = -l + t
    theQuery.polygon.vertices.append(bottom_left)

    top_left = point2d.Point2D()
    top_left.x = -l + t
    top_left.y = l + t
    theQuery.polygon.vertices.append(top_left)

    top_right = point2d.Point2D()
    top_right.x = l + t
    top_right.y = l + t
    theQuery.polygon.vertices.append(top_right)

    bottom_right = point2d.Point2D()
    bottom_right.x = l + t
    bottom_right.y = -l + t
    theQuery.polygon.vertices.append(bottom_right)

    theQuery.polygon.z = -100
    theQuery.polygon.height = 700

    theQuery.inMapFrame = True
    theQuery.fullyEncapsulated = False

    # since epoche
    theQuery.timeinterval.time_min.seconds = 1638549273
    theQuery.timeinterval.time_min.nanos = 0
    theQuery.timeinterval.time_max.seconds = 1938549273
    theQuery.timeinterval.time_max.nanos = 0

    # labels
    label = labels_with_category.LabelsWithCategory()
    label.category = "0"
    labelWithConfidence = label_pb2.Label()
    labelWithConfidence.label = "testlabel1"
    label.labels.extend([labelWithConfidence])
    theQuery.labelsWithCategory.append(label)

    grid_imgs: List[List[image.Image]] = []
    for x in range(3):
        grid_imgs.append([])
        theQuery.polygon.vertices[0].x = -l + t + x
        theQuery.polygon.vertices[1].x = -l + t + x
        theQuery.polygon.vertices[2].x = l + t + x
        theQuery.polygon.vertices[3].x = l + t + x
        for y in range(3):
            theQuery.polygon.vertices[0].y = -l + t + y
            theQuery.polygon.vertices[1].y = l + t + y
            theQuery.polygon.vertices[2].y = l + t + y
            theQuery.polygon.vertices[3].y = -l + t + y
            grid_imgs[x].append([])
            for img in stub.GetImage(theQuery):
                grid_imgs[x][y].append(img)

    return grid_imgs


if __name__ == "__main__":
    grid_img_list = query_image_grid()

    # print the results
    for x in range(len(grid_img_list)):
        for y in range(len(grid_img_list[x])):
            print(f"center point query (x/y): ( {x} / {y} )")
            print(f"Number of images: {len(grid_img_list[x][y])}")
            for img in grid_img_list[x][y]:
                print(f"Image uuid: {img.header.uuid_msgs}")
                # print( f"General label of transferred img: {img.labels_general[0].labelWithInstance[1].label.label}")
                # print( f"General label confidence: {img.labels_general[0].labelWithInstance[1].label.confidence}")
            print("--------------------")
