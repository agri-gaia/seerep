#!/usr/bin/env python3

import flatbuffers
from seerep.fb import Datatype, QueryInstance, UuidsPerProject
from seerep.fb import instance_service_grpc_fb as instanceService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createLabelsWithCategories,
    createLabelWithCategory,
    createPoint2d,
    createPolygon2D,
    createQuery,
    createQueryInstance,
    createTimeInterval,
    createTimeStamp,
    getProject,
)
from seerep.util.fb_to_dict import fb_obj_to_dict

builder = flatbuffers.Builder(1024)
# Default server is localhost !
channel = get_gRPC_channel()


# 1. Get all projects from the server
projectuuid = getProject(builder, channel, "testproject")

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    exit()

# 3. Get gRPC service object
stub = instanceService.InstanceServiceStub(channel)
print(type(stub).__name__)

# Create all necessary objects for the query
polygon_vertices = []
polygon_vertices.append(createPoint2d(builder, 0, 0))
polygon_vertices.append(createPoint2d(builder, 0, 100))
polygon_vertices.append(createPoint2d(builder, 100, 100))
polygon_vertices.append(createPoint2d(builder, 100, 0))
polygon2d = createPolygon2D(builder, 100, 0, polygon_vertices)

timeMin = createTimeStamp(builder, 1610549273, 0)
timeMax = createTimeStamp(builder, 1938549273, 0)
timeInterval = createTimeInterval(builder, timeMin, timeMax)


projectUuids = [builder.CreateString(projectuuid)]
category = ["0"]
labels = {
    "0": [
        (builder.CreateString("testlabel0"), 0.8),
        (builder.CreateString("testlabelgeneral0"), 0.9),
    ]
}

labelCategory = createLabelsWithCategories(builder, category, labels)
dataUuids = [builder.CreateString("5a0438b8-37cf-412e-8331-a95ef95c1016")]
instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = createQuery(
    builder,
    # boundingBox=boundingboxStamped,
    # timeInterval=timeInterval,
    # labels=labelCategory,
    # projectUuids=projectUuids,
    # instanceUuids=instanceUuids,
    # dataUuids=dataUuids,
    # withoutData=True,
    # fullyEncapsulated=False,
)


queryInstanceMsg = createQueryInstance(builder, query, Datatype.Datatype().All)

builder.Finish(queryInstanceMsg)
buf = builder.Output()

# print(fb_obj_to_dict(QueryInstance.QueryInstance.GetRootAs(buf)))

# catch error

responseBuf = stub.GetInstances(bytes(buf))

response = UuidsPerProject.UuidsPerProject.GetRootAs(responseBuf)

if response.UuidsPerProjectLength() > 0:
    print(response.UuidsPerProject(0).UuidsLength())
    print(response.UuidsPerProject(0).ProjectUuid().decode("utf-8"))
    # debugging
    # print(fb_obj_to_dict(response))
else:
    print("no project with that instance type found!")
