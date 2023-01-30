#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import ProjectCreation, ProjectInfos, geodeticCoordinates
from fb import image_service_grpc_fb as imageService
from fb import meta_operations_grpc_fb as metaOperations

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

channel = util.get_gRPC_channel()

stub = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

builder = flatbuffers.Builder(1024)

util_fb.createProject(channel, builder, "geodeticProject", "2", "EPSG::4326", "EPSG::7030", 4, 4, 4)

buf = builder.Output()
