# test file for
#   gRPC_fb_getTf.py
from typing import Dict, List

import flatbuffers
from gRPC.images import gRPC_fb_sendImages as send_img
from gRPC.tf import gRPC_fb_deleteTfs as del_tfs
from gRPC.util.msg_abs.msgs import (
    DatatypeImplementations,
    EnumFbQuery,
    FbQuery,
    ServiceManager,
)


def sort_call(serv_man: ServiceManager, query: FbQuery) -> List[Dict]:
    return sorted(
        serv_man.call_get_images_fb_dict(
            query.builder, query.datatype_instance
        ),
        key=lambda x: x["header"]["stamp"]["seconds"],
    )


def change_assemble_call(
    serv_man: ServiceManager, query: FbQuery, extent: float
) -> List[Dict]:
    query.set_active_function(
        EnumFbQuery.POLYGON,
        lambda: DatatypeImplementations.Fb.mod_polygon2D(
            query.builder, extent, 500
        ),
    )

    query.assemble_datatype_instance()
    return sort_call(serv_man, query)


# test sending images with tfs and deleting the tfs in part
def test_gRPC_fb_deleteTfs(grpc_channel, project_setup):
    _, project_uuid = project_setup
    timestamp_nanos = 1245
    timestamps = [
        (t, timestamp_nanos) for t in range(1661336507, 1661336608, 10)
    ]

    camera_uuid = send_img.send_cameraintrinsics(grpc_channel, project_uuid)

    # per timestamp the tf is increased by 100 times according
    # to the index of the timestamp
    send_img.send_images(
        grpc_channel, project_uuid, camera_uuid, send_img.generate_image_ressources(10), timestamps
    )

    send_img.send_tfs(grpc_channel, project_uuid, timestamps)

    builder = flatbuffers.Builder()

    # first check what happens if the first tfs are deleted
    # for testing execute a image query on that area
    query = FbQuery(
        grpc_channel,
        builder,
        {EnumFbQuery.WITHOUTDATA, EnumFbQuery.POLYGON, EnumFbQuery.PROJECTUUID},
    )

    query.set_active_function(EnumFbQuery.PROJECTUUID, lambda: [project_uuid])

    serv_man = ServiceManager(grpc_channel)

    # this value should stay this way, otherwise the other functions
    # have to be altered as well
    ts_idx = 3

    images_before = change_assemble_call(serv_man, query, 400)

    # the first 2 images shouldn't have a valid transform now
    del_tfs.delete_tfs(
        timestamps[0],
        timestamps[ts_idx],
        "map",
        "camera",
        project_uuid,
        grpc_channel,
    )

    images_after = sort_call(serv_man, query)

    assert images_after == images_before[ts_idx:]

    # if tfs in between are deleted, tfs are interpolated linearly
    # check the first 6 images (now 4 because of the deletion before)
    images_before = change_assemble_call(serv_man, query, 1600)

    # tfs at 1600 and 2500 are deleted, the same images
    # must occur at (900+900) and (900+1800) [900, 3600]
    del_tfs.delete_tfs(
        timestamps[ts_idx + 1],
        timestamps[ts_idx + 3],
        "map",
        "camera",
        project_uuid,
        grpc_channel,
    )

    images_after = sort_call(serv_man, query)
    assert images_after == images_before[:-1]

    images_after = change_assemble_call(serv_man, query, 1798)
    assert images_after == images_before[0:1]

    images_after = change_assemble_call(serv_man, query, 1800)
    assert images_after == images_before

    images_before = change_assemble_call(serv_man, query, 3600)

    images_after = change_assemble_call(serv_man, query, 2698)
    assert images_after == images_before[0:2]

    images_after = change_assemble_call(serv_man, query, 2700)
    assert images_after == images_before[0:3]

    images_before = change_assemble_call(serv_man, query, 100 * 10**2)

    # test what happens with the end edge case
    del_tfs.delete_tfs(
        timestamps[-1],
        (timestamps[-1][0] + 1, timestamps[-1][1]),
        "map",
        "camera",
        project_uuid,
        grpc_channel,
    )

    # the last image shouldn't have a valid tf
    images_after = sort_call(serv_man, query)
    assert images_after == images_before[:-1]
