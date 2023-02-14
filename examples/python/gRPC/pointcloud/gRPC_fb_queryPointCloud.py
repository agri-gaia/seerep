import os
import struct
import sys

import flatbuffers
import numpy as np

np.set_printoptions(precision=7)
from seerep.fb import PointCloud2
from seerep.fb import point_cloud_service_grpc_fb as pointCloudService

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

channel = util.get_gRPC_channel()

stubPointCloud = pointCloudService.PointCloudServiceStub(channel)
builder = flatbuffers.Builder(1024)

PROJECTNAME = "testproject"
projectUuid = util_fb.getProject(builder, channel, PROJECTNAME)

if projectUuid is None:
    print(f"Project: {PROJECTNAME} does not exist")
    sys.exit()


builder = flatbuffers.Builder(1024)

timeMin = util_fb.createTimeStamp(builder, 1610549273, 0)
timeMax = util_fb.createTimeStamp(builder, 1938549273, 0)
timeInterval = util_fb.createTimeInterval(builder, timeMin, timeMax)

category = "0"
labels = [[builder.CreateString("testlabel0"), builder.CreateString("BoundingBoxLabel0")]]
labelCategory = util_fb.createLabelWithCategory(builder, category, labels)

queryMsg = util_fb.createQuery(
    builder, projectUuids=[builder.CreateString(projectUuid)], timeInterval=timeInterval, labels=labelCategory
)
builder.Finish(queryMsg)
buf = builder.Output()

for responseBuf in stubPointCloud.GetPointCloud2(bytes(buf)):
    response = PointCloud2.GetRootAs(responseBuf)

    print("---Header---")
    print(f"Message UUID: {response.Header().UuidMsgs().decode('utf-8')}")
    print(f"Project UUID: {response.Header().UuidProject().decode('utf-8')}")
    print(f"Frame ID: {response.Header().FrameId().decode('utf-8')}")

    print("---Point Fields---")
    for i in range(response.FieldsLength()):
        print(f"Field Name: {response.Fields(i).Name().decode('utf-8')}")
        print(f"Datatype: {response.Fields(i).Datatype()}")
        print(f"Offset: {response.Fields(i).Offset()}")
        print(f"Count: {response.Fields(i).Count()}")

    print("---Bounding Box Labels---")
    for i in range(response.LabelsBbLength()):
        print(f"Label {i}: {response.LabelsBb(i).LabelWithInstance().Label().decode('utf-8')}")
        print(f"Instance {i}: {response.LabelsBb(i).LabelWithInstance().InstanceUuid().decode('utf-8')}")
        print(
            f"Bounding Box Min {i}: "
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMin().X()},"
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMin().Y()},"
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMin().Z()} "
            f"(x,y,z)"
        )
        print(
            f"Bounding Box Max {i}: "
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMax().X()},"
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMax().Y()},"
            f"{response.LabelsBb(i).BoundingBoxLabeled(0).BoundingBox().PointMax().Z()} "
            f"(x,y,z)"
        )

    print("---General Labels----")
    for i in range(response.LabelsGeneralLength()):
        print(f"Label {i}: {response.LabelsGeneral(i).Label().decode('utf-8')}")
        print(f"Instance {i}: {response.LabelsGeneral(i).InstanceUuid().decode('utf-8')}")

    print("---Data--")
    if not response.DataIsNone():
        rawData = response.DataAsNumpy()
        data = [struct.unpack('f', rawData[i : i + 4]) for i in range(0, rawData.shape[0], 4)]
        reshapedData = np.array(data).reshape(960, 1280, 4)
        print(f"Data: {reshapedData}")
