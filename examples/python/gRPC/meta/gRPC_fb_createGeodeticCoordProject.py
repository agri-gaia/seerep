#!/usr/bin/env python3
# NOTE: This file is referenced in the following mkdocs files:
#   geodetic-projects-and-queries.md
# Any changes done in here will be reflected in there
import flatbuffers
from grpc import Channel
from seerep.fb import ProjectInfo
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import createProjectRaw


def create_geo_proj_raw(
    grpc_channel: Channel = get_gRPC_channel(),
) -> bytearray:
    builder = flatbuffers.Builder(1024)

    # in the area around Gut Arenshorst
    # lat and long in decimal degree
    response = createProjectRaw(
        grpc_channel,
        builder,
        "geodeticProject",
        "2",
        "EPSG::4326",
        4,
        52.35_81_99,
        8.27_96_79,
    )

    return response


def create_geo_proj(
    grpc_channel: Channel = get_gRPC_channel(),
) -> ProjectInfo.ProjectInfo:
    return ProjectInfo.ProjectInfo.GetRootAs(create_geo_proj_raw(grpc_channel))


if __name__ == "__main__":
    create_geo_proj()
