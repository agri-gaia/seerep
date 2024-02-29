#!/usr/bin/env python3
import sys
from typing import Any, List, Tuple

import flatbuffers
from grpc import Channel
from seerep.fb import (
    Boundingbox,
    Categories,
    Datatype,
    Empty,
    Labels,
    ProjectInfos,
    TimeInterval,
)
from seerep.fb import meta_operations_grpc_fb as metaOperations
from seerep.util import fb_helper
from seerep.util.common import get_gRPC_channel


# importing util functions. Assuming that these files are in the parent dir
# examples/python/gRPC/util.py
# examples/python/gRPC/util_fb.py
def get_metadata(
    target_project_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> Tuple[Any, Any, Any, Any]:
    builder = flatbuffers.Builder(1024)

    meta_stub = metaOperations.MetaOperationsStub(grpc_channel)

    # 1. Get all projects from the server
    if target_project_uuid is None:
        Empty.Start(builder)
        empty_msg = Empty.End(builder)
        builder.Finish(empty_msg)

        buf = builder.Output()

        response_buf = meta_stub.GetProjects(bytes(buf))

        response = ProjectInfos.ProjectInfos.GetRootAs(response_buf)
        for p_idx in range(response.ProjectsLength()):
            if response.Projects(p_idx).Name().decode() == "LabeledImagesInGrid":
                target_project_uuid = response.Projects(p_idx).Uuid().decode()
        if target_project_uuid is None:
            print("No test project found, create a project with 'gRPC_pb_sendLabeledImageGrid.py' beforehand!")
            sys.exit()

    uuid_datatype_pair = fb_helper.createUuidDatatypePair(builder, target_project_uuid, Datatype.Datatype().All)

    builder.Finish(uuid_datatype_pair)
    buf = builder.Output()

    ###
    # Fetching overall spatial bound

    responseBuf = meta_stub.GetOverallBoundingBox(bytes(buf))
    response = Boundingbox.Boundingbox.GetRootAs(responseBuf)

    print(
        "Center Point (X, Y, Z): "
        + str(response.CenterPoint().X())
        + " , "
        + str(response.CenterPoint().Y())
        + " , "
        + str(response.CenterPoint().Z())
    )

    resp_overall_bb_centerp = [
        response.CenterPoint().X(),
        response.CenterPoint().Y(),
        response.CenterPoint().Z(),
    ]

    print(
        "Spatial Extent Point (X, Y, Z): "
        + str(response.SpatialExtent().X())
        + " , "
        + str(response.SpatialExtent().Y())
        + " , "
        + str(response.SpatialExtent().Z())
    )
    resp_overall_bb_spatiale = [
        response.SpatialExtent().X(),
        response.SpatialExtent().Y(),
        response.SpatialExtent().Z(),
    ]

    resp_overall_bb = [resp_overall_bb_centerp, resp_overall_bb_spatiale]

    ###
    # Fetching overall temporal bound

    responseBuf = meta_stub.GetOverallTimeInterval(bytes(buf))
    response = TimeInterval.TimeInterval.GetRootAs(responseBuf)

    resp_overall_time_intervall = (
        (response.TimeMin().Seconds(), response.TimeMin().Nanos()),
        (response.TimeMax().Seconds(), response.TimeMax().Nanos()),
    )

    ###
    # Fetching all category names

    responseBuf = meta_stub.GetAllCategories(bytes(buf))
    response = Categories.Categories.GetRootAs(responseBuf)

    resp_all_categories: List[str] = []

    print("Saved Category names are:")
    for idx in range(response.CategoriesLength()):
        print(response.Categories(idx).decode())
        resp_all_categories.append(response.Categories(idx).decode())

    ###
    # Fetching all label names for a given category

    builder = flatbuffers.Builder(1024)

    uuid_datatype_w_category = fb_helper.createUuidDatatypeWithCategory(
        builder, target_project_uuid, Datatype.Datatype().Image, "1"
    )

    builder.Finish(uuid_datatype_w_category)
    buf = builder.Output()

    responseBuf = meta_stub.GetAllLabels(bytes(buf))
    response = Labels.Labels.GetRootAs(responseBuf)

    resp_all_labels: List[str] = []
    print("Saved Label names are:")
    for idx in range(response.LabelsLength()):
        print(response.Labels(idx).decode())
        resp_all_labels.append(response.Labels(idx).decode())

    return (
        resp_overall_time_intervall,
        resp_overall_bb,
        resp_all_categories,
        resp_all_labels,
    )


if __name__ == "__main__":
    (
        resp_overall_time_intervall,
        resp_overall_bb,
        resp_all_categories,
        resp_all_labels,
    ) = get_metadata()
    print(
        f"""the overall time intervall response over all images is:
          Min: {resp_overall_time_intervall[0][0]} secs, {resp_overall_time_intervall[0][1]} nanos
          Max: {resp_overall_time_intervall[1][0]} secs, {resp_overall_time_intervall[1][1]} nanos\n
          """
    )
    print(
        f"""the overall bounding box response over all images is:
          CenterPoint (X, Y, Z): ({resp_overall_bb[0][0]}, {resp_overall_bb[0][1]}, {resp_overall_bb[0][2]})
          SpatialExtent (X, Y, Z): ({resp_overall_bb[1][0]}, {resp_overall_bb[1][1]}, {resp_overall_bb[1][2]})\n"""
    )
    print(f"the all categories response over all images is:\n{resp_all_categories}\n")
    print(f"the all labels response over all images is:\n{resp_all_labels}")
