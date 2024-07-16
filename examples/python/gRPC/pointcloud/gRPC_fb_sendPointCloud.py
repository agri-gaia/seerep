#!/user/bin/env python3
import struct
import uuid
from typing import List

import flatbuffers
import numpy as np
from grpc import Channel
from seerep.fb import (
    PointCloud2,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
    tf_service_grpc_fb,
)
from seerep.fb import point_cloud_service_grpc_fb as pointCloudService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    addToPointFieldVector,
    create_label,
    create_label_category,
    createHeader,
    createPointFields,
    createTimeStamp,
    getOrCreateProject,
)

NUM_LABELS = 10
NUM_POINT_CLOUDS = 10


def createPointCloud(builder, header, height=960, width=1280):
    """Creates a flatbuffers point cloud message"""
    pointFields = createPointFields(builder, ["x", "y", "z", "rgba"], 7, 4, 1)
    pointFieldsVector = addToPointFieldVector(builder, pointFields)

    # create labels
    labelsStrings = [f"Label{i}" for i in range(NUM_LABELS)]
    instance_uuids = [str(uuid.uuid4()) for _ in range(NUM_LABELS)]

    labels = []
    for i in range(len(labelsStrings)):
        labels.append(
            create_label(
                builder=builder,
                label=labelsStrings[i],
                label_id=1,
                instance_uuid=instance_uuids[i],
                instance_id=5,
            )
        )
    labelsCategory = []
    labelsCategory.append(
        create_label_category(
            builder=builder,
            labels=labels,
            datumaro_json="a very valid datumaro json",
            category="category Z",
        )
    )

    PointCloud2.StartLabelsVector(builder, len(labelsCategory))
    for label in labelsCategory:
        builder.PrependUOffsetTRelative(label)
    labelsCatOffset = builder.EndVector()

    # Note: rgb field is float, for simplification
    pointsBox = [[]]
    lim = 3
    for a in range(-lim, lim + 1, 1):
        for b in range(-lim, lim + 1, 1):
            for c in range(-lim, lim + 1, 1):
                pointsBox[0].append([a, b, c, 0])
    points = np.array(pointsBox).astype(np.float32)

    pointsVector = builder.CreateByteVector(points.tobytes())

    # add all data into the flatbuffers point cloud message
    PointCloud2.Start(builder)
    PointCloud2.AddHeader(builder, header)
    PointCloud2.AddHeight(builder, points.shape[0])
    PointCloud2.AddWidth(builder, points.shape[1])
    PointCloud2.AddIsBigendian(builder, True)
    PointCloud2.AddPointStep(builder, 16)
    PointCloud2.AddRowStep(builder, points.shape[1] * 16)
    PointCloud2.AddFields(builder, pointFieldsVector)
    PointCloud2.AddData(builder, pointsVector)
    PointCloud2.AddLabels(builder, labelsCatOffset)
    return PointCloud2.End(builder)


def createPointClouds(projectUuid, numOf, theTime):
    """Creates numOf pointcloud2 messages as a generator function"""

    for i in range(numOf):
        builder = flatbuffers.Builder(1024)

        timeStamp = createTimeStamp(builder, theTime + i)
        header = createHeader(
            builder, timeStamp, "scanner", projectUuid, str(uuid.uuid4())
        )

        pointCloudMsg = createPointCloud(builder, header, 10, 10)
        builder.Finish(pointCloudMsg)
        yield bytes(builder.Output())


def createTF(numOf, projectUuid, theTime):
    for i in range(numOf):
        builderTf = flatbuffers.Builder(1024)

        timeStamp = createTimeStamp(builderTf, theTime + i)
        header = createHeader(
            builderTf, timeStamp, "map", projectUuid, str(uuid.uuid4())
        )

        Vector3.Start(builderTf)
        Vector3.AddX(builderTf, 10 * i)
        Vector3.AddY(builderTf, 10 * i)
        Vector3.AddZ(builderTf, 10 * i)
        trans = Vector3.End(builderTf)

        # [ x: 45, y: 45, z: 45 ]
        Quaternion.Start(builderTf)
        Quaternion.AddX(builderTf, 0.4619398)
        Quaternion.AddY(builderTf, 0.1913417)
        Quaternion.AddZ(builderTf, 0.4619398)
        Quaternion.AddW(builderTf, 0.7325378)
        rot = Quaternion.End(builderTf)

        Transform.Start(builderTf)
        Transform.AddTranslation(builderTf, trans)
        Transform.AddRotation(builderTf, rot)
        tf = Transform.End(builderTf)

        childFrame = builderTf.CreateString("scanner")

        TransformStamped.Start(builderTf)
        TransformStamped.AddHeader(builderTf, header)
        TransformStamped.AddChildFrameId(builderTf, childFrame)
        TransformStamped.AddTransform(builderTf, tf)
        tfStamped = TransformStamped.End(builderTf)
        builderTf.Finish(tfStamped)
        yield bytes(builderTf.Output())


def send_pointcloud_raw(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[bytearray]:
    builder = flatbuffers.Builder(1024)
    theTime = 1686038855

    if target_proj_uuid is None:
        target_proj_uuid = getOrCreateProject(
            builder, grpc_channel, "testproject"
        )

    tfStub = tf_service_grpc_fb.TfServiceStub(grpc_channel)
    tfStub.TransferTransformStamped(
        createTF(NUM_POINT_CLOUDS, target_proj_uuid, theTime)
    )

    stub = pointCloudService.PointCloudServiceStub(grpc_channel)
    pcl = createPointClouds(target_proj_uuid, NUM_POINT_CLOUDS, theTime)

    pc_list: List = list(pcl)

    # one could directly pass the generator, e.g.
    # responseBuf = stub.TransferPointCloud2(pc)
    # but this would consume the generator and then the pc list cannot be
    # returned, therefore use a iterator
    stub.TransferPointCloud2(iter(pc_list))

    return pc_list


def send_pointcloud(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[PointCloud2.PointCloud2]:
    return [
        PointCloud2.PointCloud2.GetRootAs(p)
        for p in send_pointcloud_raw(target_proj_uuid, grpc_channel)
    ]


if __name__ == "__main__":
    np.set_printoptions(precision=7)
    pc_list = send_pointcloud()
    for idx, pc in enumerate(pc_list):
        print("Num of pc: ", idx + 1)
        raw_data = pc.DataAsNumpy()
        data_flattened = [
            struct.unpack("f", pc.DataAsNumpy()[i : i + pc.FieldsLength()])
            for i in range(0, raw_data.shape[0], pc.FieldsLength())
        ]
        # reshape the array according to its dimensions
        data = np.array(data_flattened).reshape(
            pc.Height(), pc.Width(), pc.FieldsLength()
        )

        print(f"Data: {data}")
