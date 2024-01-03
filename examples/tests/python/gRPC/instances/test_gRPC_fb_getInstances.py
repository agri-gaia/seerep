import time
from typing import Dict, List

from gRPC.images import gRPC_pb_sendLabeledImage as send_imgs
from gRPC.point import gRPC_fb_sendPointsBasedOnImageInstances as send_points
from gRPC.pointcloud import gRPC_fb_sendPointCloud as send_pcl
from seerep.fb import Datatype, PointCloud2, PointStamped, UuidsPerProject
from seerep.util import fb_helper as fbh
from seerep.util.helper.msgs import (
    EnumFbQuery,
    EnumFbQueryInstance,
    FbQuery,
    FbQueryInstance,
)
from seerep.util.helper.service_manager import ServiceManager


def get_sorted_uuids_per_proj(uuidspp: UuidsPerProject.UuidsPerProject) -> List[str]:
    if uuidspp.UuidsPerProjectLength() == 0:
        return list()
    return sorted(
        [
            uuidspp.UuidsPerProject(0).Uuids(i).decode()
            for i in range(uuidspp.UuidsPerProject(0).UuidsLength())
        ]
    )


def test_gRPC_getInstanceTypes(grpc_channel, project_setup):
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

    ### check for all instances
    queryinst_builder.set_active_function(
        EnumFbQueryInstance.DATATYPE, lambda: Datatype.Datatype.All
    )
    queryinst_builder.assemble_datatype_instance()

    instance_uuidspp = serv_man.call_get_instances_fb(
        queryinst_builder.builder, queryinst_builder.datatype_instance
    )
    instance_uuids = get_sorted_uuids_per_proj(instance_uuidspp)

    # because uuid duplicates are received
    instance_uuids = sorted(list(set(instance_uuids)))

    points_instance_uuids.extend(pcl_instance_uuids)
    points_instance_uuids.extend(bbs_instances)

    instance_uuids_all = sorted(list(set(points_instance_uuids)))

    assert instance_uuids == instance_uuids_all


def combine_stamps_nanos(pb_obj):
    return pb_obj.header.stamp.seconds * 10**9 + pb_obj.header.stamp.nanos


def get_instances_from_imgs(img_lst: List):
    bbs_instances = []
    for image in img_lst:
        for bb_cat in image.labels_bb:
            for instance in bb_cat.boundingBox2DLabeled:
                bbs_instances.append(instance.labelWithInstance.instanceUuid)
    return bbs_instances


def test_gRPC_getInstanceQueryPolygon(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    serv_man = ServiceManager(grpc_channel)

    # only send pictures to ease the testing process
    images_uuids, _, _ = send_imgs.send_labeled_images(proj_uuid, grpc_channel)

    # extract images ignore image uuids
    images = [img[1] for img in images_uuids]

    timestamp_sorted_imgs = sorted(images, key=lambda img: combine_stamps_nanos(img))
    timestamp_thresh_imgs = timestamp_sorted_imgs[:4]

    # retrieve the labelinstances of the bounding boxes
    bbs_instances = get_instances_from_imgs(timestamp_thresh_imgs)

    query_builder = FbQuery(grpc_channel, enum_types=set([EnumFbQuery.POLYGON]))
    queryinst_builder = FbQueryInstance(
        grpc_channel, enum_types=set([EnumFbQueryInstance.QUERY])
    )
    queryinst_builder.set_active_function(
        EnumFbQueryInstance.QUERY, lambda: query_builder.datatype_instance
    )

    queryinst_builder.assemble_datatype_instance()

    instance_uuids_polygon = get_sorted_uuids_per_proj(
        serv_man.call_get_instances_fb(
            queryinst_builder.builder, queryinst_builder.datatype_instance
        )
    )

    assert instance_uuids_polygon == sorted(bbs_instances)


def test_gRPC_getInstanceQueryTimeinterval(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    serv_man = ServiceManager(grpc_channel)

    # only send pictures to ease the testing process
    images_uuids, _, _ = send_imgs.send_labeled_images(proj_uuid, grpc_channel)

    # extract images ignore image uuids
    images = [img[1] for img in images_uuids]

    ### time interval tests
    time_offset = 1000
    cur_time = int(time.time())

    min_time_ = cur_time - time_offset
    max_time_ = cur_time + time_offset

    query_builder = FbQuery(grpc_channel, enum_types=set([EnumFbQuery.TIMEINTERVAL]))
    queryinst_builder = FbQueryInstance(
        grpc_channel, enum_types=set([EnumFbQueryInstance.QUERY])
    )

    # test for time interval
    min_timestamp = fbh.createTimeStamp(queryinst_builder.builder, min_time_, 0)
    max_timestamp = fbh.createTimeStamp(queryinst_builder.builder, max_time_, 0)

    img_time_interval = fbh.createTimeInterval(
        queryinst_builder.builder, min_timestamp, max_timestamp
    )

    query_builder.set_active_function(
        EnumFbQuery.TIMEINTERVAL, lambda: img_time_interval
    )

    queryinst_builder.set_active_function(
        EnumFbQueryInstance.QUERY, lambda: query_builder.datatype_instance
    )

    query_builder.assemble_datatype_instance()

    queryinst_builder.assemble_datatype_instance()

    instance_uuids_intimeinterval = get_sorted_uuids_per_proj(
        serv_man.call_get_instances_fb(
            queryinst_builder.builder, queryinst_builder.datatype_instance
        )
    )

    min_time_ = cur_time - 2 * time_offset
    max_time_ = cur_time - time_offset

    min_timestamp = fbh.createTimeStamp(queryinst_builder.builder, min_time_, 0)
    max_timestamp = fbh.createTimeStamp(queryinst_builder.builder, max_time_, 0)

    img_time_interval = fbh.createTimeInterval(
        queryinst_builder.builder, min_timestamp, max_timestamp
    )

    query_builder.set_active_function(
        EnumFbQuery.TIMEINTERVAL, lambda: img_time_interval
    )

    query_builder.assemble_datatype_instance()

    queryinst_builder.assemble_datatype_instance()

    instance_uuids_outtimeinterval = get_sorted_uuids_per_proj(
        serv_man.call_get_instances_fb(
            queryinst_builder.builder, queryinst_builder.datatype_instance
        )
    )

    bbs_instances = get_instances_from_imgs(images)

    assert len(instance_uuids_outtimeinterval) == 0
    assert sorted(bbs_instances) == sorted(instance_uuids_intimeinterval)


# this test retrieves all instances with the label testlabel0/testlabelgeneral0
# in the current state this fails
def _test_gRPC_getInstanceQueryLabel(grpc_channel, project_setup):
    filter_label = "testlabel0"
    _, proj_uuid = project_setup

    serv_man = ServiceManager(grpc_channel)

    # only send pictures to ease the testing process
    images_uuids, _, _ = send_imgs.send_labeled_images(proj_uuid, grpc_channel)

    # extract images ignore image uuids
    images = [img[1] for img in images_uuids]

    query_builder = FbQuery(grpc_channel, enum_types=set([EnumFbQuery.LABEL]))
    queryinst_builder = FbQueryInstance(
        grpc_channel, enum_types=set([EnumFbQueryInstance.QUERY])
    )

    queryinst_builder.set_active_function(
        EnumFbQueryInstance.QUERY, lambda: query_builder.datatype_instance
    )

    query_builder.assemble_datatype_instance()
    queryinst_builder.assemble_datatype_instance()

    instance_uuids = get_sorted_uuids_per_proj(
        serv_man.call_get_instances_fb(
            queryinst_builder.builder, queryinst_builder.datatype_instance
        )
    )

    bbs_instances = []
    for image in images:
        for bb_cat in image.labels_bb:
            for instance in bb_cat.boundingBox2DLabeled:
                if instance.labelWithInstance.label.label == filter_label:
                    bbs_instances.append(instance.labelWithInstance.instanceUuid)

    bbs_instances = sorted(bbs_instances)

    assert instance_uuids == bbs_instances


# set breakpoint in seerep_srv/seerep_core_fb/src/core_fb_instance.cpp line 22
def _test_gRPC_getInstanceQueryInstanceUuid(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    serv_man = ServiceManager(grpc_channel)

    # only send pictures to ease the testing process
    images_uuids, _, _ = send_imgs.send_labeled_images(proj_uuid, grpc_channel)

    # extract images ignore image uuids
    images = [img[1] for img in images_uuids]

    # retrieve the labelinstances of the bounding boxes
    bbs_instances = sorted(get_instances_from_imgs(images))

    query_builder = FbQuery(
        grpc_channel,
        enum_types=set([EnumFbQuery.PROJECTUUID, EnumFbQuery.INSTANCEUUID]),
    )
    query_builder.set_active_function(
        EnumFbQuery.PROJECTUUID, lambda: [query_builder.builder.CreateString(proj_uuid)]
    )
    query_builder.assemble_datatype_instance()

    queryinst_builder = FbQueryInstance(
        grpc_channel, enum_types=set([EnumFbQueryInstance.QUERY])
    )
    queryinst_builder.set_active_function(
        EnumFbQueryInstance.QUERY, lambda: query_builder.datatype_instance
    )

    queryinst_builder.assemble_datatype_instance()

    print(len(query_builder.get_component(EnumFbQuery.INSTANCEUUID)))

    instance_uuids = get_sorted_uuids_per_proj(
        serv_man.call_get_instances_fb(
            queryinst_builder.builder, queryinst_builder.datatype_instance
        )
    )
    print(len(instance_uuids))

    bbs_uuids_halved = [bbs_instances[i] for i in range(0, len(instance_uuids), 2)]

    assert instance_uuids == sorted(bbs_uuids_halved)
