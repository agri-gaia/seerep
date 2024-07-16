#!/usr/bin/env python3
NUM_BB_LABELS = 5

import sys
import uuid
from typing import List, Tuple

import flatbuffers
from grpc import Channel
from seerep.fb import DatasetUuidLabel, PointCloud2
from seerep.fb import point_cloud_service_grpc_fb as pointcloudService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    create_dataset_uuid_label,
    create_label,
    create_label_category,
    createQuery,
    getProject,
)


def add_pc_label_raw(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[Tuple[str, bytearray]]:
    builder = flatbuffers.Builder(1024)

    if target_proj_uuid is None:
        target_proj_uuid = getProject(builder, grpc_channel, "testproject")

        if target_proj_uuid is None:
            print("Requested project does not exist")
            sys.exit()

    stub = pointcloudService.PointCloudServiceStub(grpc_channel)

    query = createQuery(
        builder,
        projectUuids=[builder.CreateString(target_proj_uuid)],
        withoutData=True,
    )
    builder.Finish(query)
    buf = builder.Output()

    msgToSend = []
    label_list: List[Tuple[str, bytearray]] = []

    for responseBuf in stub.GetPointCloud2(bytes(buf)):
        response = PointCloud2.PointCloud2.GetRootAs(responseBuf)

        pc_uuid = response.Header().UuidMsgs().decode("utf-8")
        projectUuid = response.Header().UuidProject().decode("utf-8")

        labelStr = ["label1", "label2"]
        labels = []

        for labelAct in labelStr:
            labels.append(
                create_label(
                    builder=builder,
                    label=labelAct,
                    label_id=1,
                    instance_uuid=str(uuid.uuid4()),
                    instance_id=2,
                )
            )
        labelsCategory = []
        labelsCategory.append(
            create_label_category(
                builder=builder,
                labels=labels,
                datumaro_json="a very valid datumaro json",
                category="laterAddedLabel",
            )
        )

        dataset_uuid_label = create_dataset_uuid_label(
            builder=builder,
            projectUuid=projectUuid,
            datasetUuid=pc_uuid,
            labels=labelsCategory,
        )

        builder.Finish(dataset_uuid_label)
        buf = builder.Output()

        label_list.append(
            (
                pc_uuid,
                buf,
            )
        )

        msgToSend.append(bytes(buf))

    response = stub.AddLabels(iter(msgToSend))
    return label_list


def add_pc_label(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[Tuple[str, DatasetUuidLabel.DatasetUuidLabel]]:
    return [
        (pc_uuid, DatasetUuidLabel.DatasetUuidLabel.GetRootAs(resp_buf))
        for pc_uuid, resp_buf in add_pc_label_raw(
            target_proj_uuid, grpc_channel
        )
    ]


if __name__ == "__main__":
    label_list = add_pc_label()

    for pc_uuid, labelAllCat in label_list:
        print(f"Added label to pc with uuid {pc_uuid}:")
        for labelCategory in labelAllCat.Labels():
            for label in labelCategory.Labels():
                print(f"uuid: {label.Label().decode()}")
