#!/usr/bin/env python3
# NOTE: This file is referenced in the following mkdocs files:
#   images.md
# Any changes done in here will be reflected in there
import sys
from typing import List, Optional

from google.protobuf import empty_pb2
from grpc import Channel
from seerep.pb import image_pb2 as image
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_category_pb2, label_pb2
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import point2d_pb2 as point2d
from seerep.pb import query_pb2 as query
from seerep.util.common import get_gRPC_channel


def query_images(
    target_project_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> List[image.Image]:
    # 1. Get gRPC service objects
    stub = imageService.ImageServiceStub(grpc_channel)
    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)

    # 3. Check if we have an existing test project, if not, we stop here
    if target_project_uuid is None:
        # 2. Get all projects from the server
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            print(project.name + " " + project.uuid + "\n")
            if project.name == "testproject":
                target_project_uuid = project.uuid

        if target_project_uuid is None:
            print("""
                No project with name 'testproject' found! Execute
                gRPC_pb_sendLabeledImage.py beforehand!
            """)
            sys.exit()

    # 4. Create a query with parameters
    theQuery = query.Query()
    theQuery.projectuuid.append(target_project_uuid)

    theQuery.polygon.z = -200
    theQuery.polygon.height = 800

    theQuery.inMapFrame = True
    theQuery.crsString = "EPSG:4326"

    scale = 150
    vertices = [
        point2d.Point2D(x=x, y=y)
        for x, y in [
            (-scale, -scale),
            (-scale, scale),
            (scale, scale),
            (scale, -scale),
        ]
    ]
    theQuery.polygon.vertices.extend(vertices)

    # since epoche
    theQuery.timeinterval.time_min.seconds = 1638549273
    theQuery.timeinterval.time_min.nanos = 0
    theQuery.timeinterval.time_max.seconds = 1938549273
    theQuery.timeinterval.time_max.nanos = 0

    # labels
    labelsCategory = label_category_pb2.LabelCategory()
    labelsCategory.category = "category A"
    label = label_pb2.Label()
    label.label = "label1"
    labelsCategory.labels.append(label)
    theQuery.labelCategory.append(labelsCategory)

    # theQuery.inMapFrame = True
    theQuery.fullyEncapsulated = False

    # 5. Query the server for images matching the query and return them
    return list(stub.GetImage(theQuery))


if __name__ == "__main__":
    queried_imgs = query_images()
    print(f"count of images {len(queried_imgs)}")
    for img in queried_imgs:
        print(
            f"uuidmsg: {img.header.uuid_msgs}"
            + "\n"
            + f"first label: {img.labels[0].labels[0].label}"
        )
