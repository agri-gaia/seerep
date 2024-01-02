from typing import Dict, List

from gRPC.images import gRPC_pb_sendLabeledImage as send_imgs
from gRPC.point import gRPC_fb_sendPointsBasedOnImageInstances as send_points
from gRPC.pointcloud import gRPC_fb_sendPointCloud as send_pcl
from seerep.fb import Datatype, PointCloud2, PointStamped, UuidsPerProject
from seerep.util.helper.msgs import EnumFbQueryInstance, FbQueryInstance
from seerep.util.helper.service_manager import ServiceManager


def get_sorted_uuids_per_proj(uuidspp: UuidsPerProject.UuidsPerProject) -> List[str]:
    return sorted(
        [
            uuidspp.UuidsPerProject(0).Uuids(i).decode()
            for i in range(uuidspp.UuidsPerProject(0).UuidsLength())
        ]
    )


def test_gRPC_getInstances(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    ### check for instances on images
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

    queryinst_builder = FbQueryInstance(
        grpc_channel, enum_types=set([EnumFbQueryInstance.DATATYPE])
    )
    queryinst_builder.set_active_function(
        EnumFbQueryInstance.DATATYPE, lambda: Datatype.Datatype.Image
    )
    queryinst_builder.assemble_datatype_instance()

    instance_uuidspp = serv_man.call_get_instances_fb(
        queryinst_builder.builder, queryinst_builder.datatype_instance
    )

    bbs_instances = sorted(bbs_instances)

    instance_uuids = get_sorted_uuids_per_proj(instance_uuidspp)

    assert instance_uuids == bbs_instances

    ### check for instances on poinclouds
    pcl_lst: List[PointCloud2.PointCloud2] = send_pcl.send_pointcloud(
        proj_uuid, grpc_channel
    )
    pcl_instance_uuids = []

    # extract general labels, which count in this case as instances
    for pcl in pcl_lst:
        for i in range(pcl.LabelsGeneralLength()):
            for j in range(pcl.LabelsGeneral(i).LabelsWithInstanceLength()):
                pcl_instance_uuids.append(
                    pcl.LabelsGeneral(i).LabelsWithInstance(j).InstanceUuid().decode()
                )

    queryinst_builder.set_active_function(
        EnumFbQueryInstance.DATATYPE, lambda: Datatype.Datatype.PointCloud
    )
    queryinst_builder.assemble_datatype_instance()

    instance_uuidspp = serv_man.call_get_instances_fb(
        queryinst_builder.builder, queryinst_builder.datatype_instance
    )

    pcl_instance_uuids = sorted(pcl_instance_uuids)
    instance_uuids = get_sorted_uuids_per_proj(instance_uuidspp)

    assert pcl_instance_uuids == instance_uuids

    ### check for instances on points
    img_uuid2point_map: Dict[str, PointStamped.PointStamped] = send_points.send_points(
        proj_uuid, grpc_channel
    )
    point_lst = [p for pl in img_uuid2point_map.values() for p in pl]

    points_imguuids = set()
    for p in point_lst:
        for i in range(p.LabelsGeneralLength()):
            for j in range(p.LabelsGeneral(i).LabelsWithInstanceLength()):
                points_imguuids.add(
                    p.LabelsGeneral(i).LabelsWithInstance(j).InstanceUuid().decode()
                )

    queryinst_builder.set_active_function(
        EnumFbQueryInstance.DATATYPE, lambda: Datatype.Datatype.Point
    )
    queryinst_builder.assemble_datatype_instance()

    instance_uuidspp = serv_man.call_get_instances_fb(
        queryinst_builder.builder, queryinst_builder.datatype_instance
    )

    points_instance_uuids = sorted(list(points_imguuids))
    instance_uuids = get_sorted_uuids_per_proj(instance_uuidspp)

    assert points_instance_uuids == instance_uuids
