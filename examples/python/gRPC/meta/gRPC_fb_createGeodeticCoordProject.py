#!/usr/bin/env python3

import flatbuffers
from seerep.fb import image_service_grpc_fb as imageService
from seerep.fb import meta_operations_grpc_fb as metaOperations
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import createProject

channel = get_gRPC_channel()

stub = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

builder = flatbuffers.Builder(1024)

createProject(channel, builder, "geodeticProject", "2", "EPSG::4326", "EPSG::7030", 4, 4, 4)

buf = builder.Output()
