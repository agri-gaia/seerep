#!/usr/bin/env python3
import sys
from typing import Any, Tuple

import seerep.pb.datatype_pb2 as datatype
import seerep.pb.meta_operations_pb2_grpc as metaOperations
import seerep.pb.uuid_datatype_pair_pb2 as uuid_datatype_pair
import seerep.pb.uuid_datatype_with_category_pb2 as uuid_datatype_with_category
from google.protobuf import empty_pb2
from grpc import Channel
from seerep.util.common import get_gRPC_channel


def get_metadata(
    target_project_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> Tuple[Any, Any, Any, Any]:
    stub = metaOperations.MetaOperationsStub(grpc_channel)

    response = stub.GetProjects(empty_pb2.Empty())

    if target_project_uuid is None:
        # Check if we have an existing test project, if not, we stop here
        for project in response.projects:
            print(project.name + " " + project.uuid + "\n")
            if project.name == "LabeledImagesInGrid":
                target_project_uuid = project.uuid
        if target_project_uuid is None:
            print("No test project found, create a project with 'gRPC_pb_sendLabeledImageGrid.py' beforehand!")
            sys.exit()

    uuiddt = uuid_datatype_pair.UuidDatatypePair()
    uuiddt.projectuuid = target_project_uuid
    uuiddt.datatype = datatype.image

    resp_overall_time_intervall = stub.GetOverallTimeInterval(uuiddt)

    resp_overall_bb = stub.GetOverallBoundingBox(uuiddt)

    resp_all_categories = stub.GetAllCategories(uuiddt)

    uuiddt_category = uuid_datatype_with_category.UuidDatatypeWithCategory()
    uuiddt_category.category = "category A"
    uuiddt_category.uuid_with_datatype.projectuuid = target_project_uuid
    uuiddt_category.uuid_with_datatype.datatype = datatype.all

    resp_all_labels = stub.GetAllLabels(uuiddt_category)

    return (
        resp_overall_time_intervall,
        resp_overall_bb,
        resp_all_categories,
        resp_all_labels,
    )


if __name__ == "__main__":
    ret = get_metadata()

    print(f"the overall time intervall response over all images is:\n{ret[0]}")
    print(f"the overall bounding box response over all images is:\n{ret[1]}")
    print(f"the all categories response over all images is:\n{ret[2]}")
    print(f"the all labels response over all images is:\n{ret[3]}")
