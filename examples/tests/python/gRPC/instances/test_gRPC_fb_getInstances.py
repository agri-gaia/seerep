from typing import Final

from gRPC.images import gRPC_pb_sendLabeledImage as send_imgs
from seerep.fb import Datatype
from seerep.util.helper.msgs import (
    EnumFbQuery,
    EnumFbQueryInstance,
    FbQuery,
    FbQueryInstance,
)
from seerep.util.helper.service_manager import ServiceManager


def test_gRPC_getInstances(grpc_channel, project_setup):
    _, proj_uuid = project_setup
    images_uuids, _, _ = send_imgs.send_labeled_images(proj_uuid, grpc_channel)

    # extract images ignore image uuids
    images = [img[1] for img in images_uuids]

    # retrieve the labelinstances of the bounding boxes
    bbs_instances = []
    for image in images:
        for bb_cat in image.labels_bb:
            for instance in bb_cat.boundingBox2DLabeled:
                bbs_instances.append(instance.labelWithInstance.instanceUuid)

    serv_man = ServiceManager(grpc_channel)

    print(bbs_instances)
    queryinst_builder = FbQueryInstance(
        grpc_channel, enum_types=set([EnumFbQueryInstance.DATATYPE])
    )
    queryinst_builder.set_active_function(
        EnumFbQueryInstance.DATATYPE, lambda: Datatype.Datatype.Image
    )
    queryinst_builder.assemble_datatype_instance()

    instance_uuids = serv_man.call_get_instances_fb(
        queryinst_builder.builder, queryinst_builder.datatype_instance
    )

    assert len(
        [
            instance_uuids.UuidsPerProject(0).Uuids(i)
            for i in range(instance_uuids.UuidsPerProject(0).UuidsLength())
        ]
    ) == len(bbs_instances)
