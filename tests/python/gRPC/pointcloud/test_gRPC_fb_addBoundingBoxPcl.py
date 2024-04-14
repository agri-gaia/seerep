# test file for
#   gRPC_fb_addBoundingBox.py (Pointcloud)
# requires:
#   gRPC_fb_sendPointCloud.py
#   gRPC_fb_queryPointCloud.py
from gRPC.pointcloud import gRPC_fb_addBoundingBox as add_bb
from gRPC.pointcloud import gRPC_fb_queryPointCloud as query_pc
from gRPC.pointcloud import gRPC_fb_sendPointCloud as send_pc
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def test_addBoundingBoxToPCs(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    # send pointclouds to the project
    send_pc.send_pointcloud(proj_uuid, grpc_channel)

    # add bbs to the point clouds
    bbs_ls = sorted(
        [
            fb_flatc_dict(bb, SchemaFileNames.BOUNDINGBOXES_LABELED_STAMPED)
            for bb in add_bb.add_pc_bounding_boxes_raw(proj_uuid, grpc_channel)
        ],
        key=lambda bb: bb["header"]["uuid_msgs"],
    )

    # query pointclouds
    queried_pcs = sorted(
        [fb_flatc_dict(pcl, SchemaFileNames.POINT_CLOUD_2) for pcl in query_pc.query_pcs_raw(proj_uuid, grpc_channel)],
        key=lambda pcl: pcl["header"]["uuid_msgs"],
    )

    default_rot = {"w": 1.0}

    # sort inner bb list and add rotation to the values as this is set with default values by the server
    for bbs in bbs_ls:
        for label in bbs["labels_bb"]:
            label["boundingBoxLabeled"].sort(key=lambda bb: bb["labelWithInstance"]["instanceUuid"])
            for bb in label["boundingBoxLabeled"]:
                bb["bounding_box"]["rotation"] = default_rot

    for bbs in queried_pcs:
        for label in bbs["labels_bb"]:
            label["boundingBoxLabeled"].sort(key=lambda bb: bb["labelWithInstance"]["instanceUuid"])

    assert len(queried_pcs) == len(bbs_ls)

    for idx, pc in enumerate(queried_pcs):
        # filter for labels in category laterAddedBB
        pc_labeled_bb_with_category = [elem for elem in pc["labels_bb"] if elem["category"] == "laterAddedBB"]
        assert pc["header"]["uuid_msgs"] == bbs_ls[idx]["header"]["uuid_msgs"]
        print(pc_labeled_bb_with_category)
        print(bbs_ls[idx]["labels_bb"])
        assert pc_labeled_bb_with_category == bbs_ls[idx]["labels_bb"]
