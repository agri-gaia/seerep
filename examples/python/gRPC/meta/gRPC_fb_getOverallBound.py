#!/usr/bin/env python3

import os
import sys

import flatbuffers
from seerep.fb import Boundingbox, Categories, Datatype, Labels, TimeInterval
from seerep.fb import meta_operations_grpc_fb as metaOperations

# importing util functions. Assuming that these files are in the parent dir
# examples/python/gRPC/util.py
# examples/python/gRPC/util_fb.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)
# Default server is localhost !
channel = util.get_gRPC_channel()


# 1. Get all projects from the server
projectuuid = util_fb.getProject(builder, channel, 'LabeledImagesInGrid')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    print("Project Not Found")
    exit()

# 3. Get gRPC service object
stub = metaOperations.MetaOperationsStub(channel)

UuidDatatypePair = util_fb.createUuidDatatypePair(builder, projectuuid, Datatype.Datatype().All)

builder.Finish(UuidDatatypePair)
buf = builder.Output()

###
# Fetching overall spatial bound

responseBuf = stub.GetOverallBoundingBox(bytes(buf))
response = Boundingbox.Boundingbox.GetRootAs(responseBuf)

print(
    "Center Point (X, Y, Z): "
    + str(response.CenterPoint().X())
    + " , "
    + str(response.CenterPoint().Y())
    + " , "
    + str(response.CenterPoint().Z())
)

print(
    "Spatial Extent Point (X, Y, Z): "
    + str(response.SpatialExtent().X())
    + " , "
    + str(response.SpatialExtent().Y())
    + " , "
    + str(response.SpatialExtent().Z())
)

###
# Fetching overall temporal bound

responseBuf = stub.GetOverallTimeInterval(bytes(buf))
response = TimeInterval.TimeInterval.GetRootAs(responseBuf)

print(
    " Minimum Time "
    + str(response.TimeMin().Seconds())
    + "s and "
    + str(response.TimeMin().Nanos())
    + "ms\n"
    + " Maximum Time "
    + str(response.TimeMax().Seconds())
    + "s and "
    + str(response.TimeMax().Nanos())
    + "ms"
)

###
# Fetching all category names

responseBuf = stub.GetAllCategories(bytes(buf))
response = Categories.Categories.GetRootAs(responseBuf)

print("Saved Category names are:")
for idx in range(response.CategoriesLength()):
    print(response.Categories(idx).decode())

###
# Fetching all label names for a given category

builder = flatbuffers.Builder(1024)

UuidDatatypeWithCategory = util_fb.createUuidDatatypeWithCategory(builder, projectuuid, Datatype.Datatype().Image, "1")

builder.Finish(UuidDatatypeWithCategory)
buf = builder.Output()

responseBuf = stub.GetAllLabels(bytes(buf))
response = Labels.Labels.GetRootAs(responseBuf)

print("Saved Label names are:")
for idx in range(response.LabelsLength()):
    print(response.Labels(idx).decode())
