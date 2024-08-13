#!/usr/bin/env python3
# NOTE: This file is referenced in the following mkdocs files:
#   writing-python-examples.md
# If any line changes on this file occur, those files may have to be updated as
# well

NUM_BB_LABELS = 5

import sys
import uuid
from typing import List, Optional, Tuple

import flatbuffers
from grpc import Channel
from seerep.fb import DatasetUuidLabel, Image, ProjectInfos
from seerep.fb import image_service_grpc_fb as imageService
from seerep.fb import meta_operations_grpc_fb as meta_ops
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    create_dataset_uuid_label,
    create_label,
    create_label_category,
    createEmpty,
    createQuery,
)


def add_label_raw(
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> List[Tuple[str, bytearray]]:
    stubMeta = meta_ops.MetaOperationsStub(grpc_channel)

    # 3. Check if we have an existing test project, if not, one is created.
    if target_proj_uuid is None:
        # 2. Get all projects from the server
        response = ProjectInfos.ProjectInfos.GetRootAs(
            stubMeta.GetProjects(bytes(createEmpty(flatbuffers.Builder(1024))))
        )
        for i in range(response.ProjectsLength()):
            proj_name = response.Projects(i).Name().decode()
            proj_uuid = response.Projects(i).Uuid().decode()
            print(proj_name + " " + proj_uuid)
            if proj_name == "testproject":
                target_proj_uuid = proj_uuid

        if target_proj_uuid is None:
            print("""
                Please create a project with labeled images using
                gRPC_pb_sendLabeledImage.py first."
            """)
            sys.exit()

    stub = imageService.ImageServiceStub(grpc_channel)

    builder = flatbuffers.Builder(1024)
    query = createQuery(
        builder,
        projectUuids=[target_proj_uuid],
        withoutData=True,
    )
    builder.Finish(query)
    buf = builder.Output()

    response_ls: List = list(stub.GetImage(bytes(buf)))
    if not response_ls:
        print("""
            No images found. Please create a project with labeled images
            using gRPC_pb_sendLabeledImage.py first.
        """)
        sys.exit()

    msgToSend = []
    label_list: List[Tuple[str, bytearray]] = []

    for responseBuf in response_ls:
        response = Image.Image.GetRootAs(responseBuf)

        img_uuid = response.Header().UuidMsgs().decode("utf-8")
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
            datasetUuid=img_uuid,
            labels=labelsCategory,
        )

        builder.Finish(dataset_uuid_label)
        buf = builder.Output()

        label_list.append(
            (
                img_uuid,
                buf,
            )
        )

        msgToSend.append(bytes(buf))

    stub.AddLabels(iter(msgToSend))
    return label_list


def add_label(
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> List[Tuple[str, DatasetUuidLabel.DatasetUuidLabel]]:
    return [
        (img_uuid, DatasetUuidLabel.DatasetUuidLabel.GetRootAs(labelbuf))
        for img_uuid, labelbuf in add_label_raw(target_proj_uuid, grpc_channel)
    ]


if __name__ == "__main__":
    label_list = add_label()
    for img_uuid, labelAllCat in label_list:
        print(f"Added label to image with uuid {img_uuid}:")
        for labelCategory in labelAllCat.Labels():
            for label in labelCategory.Labels():
                print(f"uuid: {label.Label().decode()}")
