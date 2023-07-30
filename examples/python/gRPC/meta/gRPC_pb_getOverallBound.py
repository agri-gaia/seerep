#!/usr/bin/env python3
from typing import Any, Tuple

import seerep.pb.datatype_pb2 as datatype
import seerep.pb.meta_operations_pb2_grpc as metaOperations
import seerep.pb.uuid_datatype_pair_pb2 as uuid_datatype_pair
import seerep.pb.uuid_datatype_with_category_pb2 as uuid_datatype_with_category
from google.protobuf import empty_pb2
from seerep.util.common import get_gRPC_channel


def get_metadata(
    grpc_channel=get_gRPC_channel(), target_project_uuid=None
) -> Tuple[Any, Any, Any, Any]:
    stub = metaOperations.MetaOperationsStub(grpc_channel)

    response = stub.GetProjects(empty_pb2.Empty())

    if not target_project_uuid:
        # Check if we have an existing test project, if not, we stop here
        for project in response.projects:
            print(project.name + " " + project.uuid + "\n")
            if project.name == "LabeledImagesInGrid":
                target_project_uuid = project.uuid

    uuiddt = uuid_datatype_pair.UuidDatatypePair()
    uuiddt.projectuuid = target_project_uuid
    uuiddt.datatype = datatype.image

    resp_overall_time_intervall = stub.GetOverallTimeInterval(uuiddt)
    print(resp_overall_time_intervall)

    resp_overall_bb = stub.GetOverallBoundingBox(uuiddt)
    print(resp_overall_bb)

    resp_all_categories = stub.GetAllCategories(uuiddt)
    print(resp_all_categories)

    uuiddt_category = uuid_datatype_with_category.UuidDatatypeWithCategory()
    uuiddt_category.category = "1"
    uuiddt_category.uuid_with_datatype.projectuuid = target_project_uuid
    uuiddt_category.uuid_with_datatype.datatype = datatype.all

    resp_all_labels = stub.GetAllLabels(uuiddt_category)
    print(resp_all_labels)

    return (
        resp_overall_time_intervall,
        resp_overall_bb,
        resp_all_categories,
        resp_all_labels,
    )


if __name__ == "__main__":
    get_metadata()
