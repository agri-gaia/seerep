# test file for
#   gRPC_fb_addBoundingBox.py (Pointcloud)
# requires:
#   gRPC_fb_sendPointCloud.py
#   gRPC_fb_queryPointCloud.py
from gRPC.pointcloud import gRPC_fb_addBoundingBox as add_bb
from gRPC.pointcloud import gRPC_fb_queryPointCloud as query_pc
from gRPC.pointcloud import gRPC_fb_sendPointCloud as send_pc
from seerep.util import fb_to_dict


def test_addBoundingBoxToPCs(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    # send pointclouds to the project
    send_pc.send_pointcloud(proj_uuid, grpc_channel)

    # add bbs to the point clouds
    bbs_ls = [
        fb_to_dict.fb_obj_to_dict(elem)
        for elem in sorted(
            add_bb.add_pc_bounding_boxes(proj_uuid, grpc_channel),
            key=lambda bb: bb.Header().UuidMsgs().decode(),
        )
    ]

    # query pointclouds
    queried_pcs = [
        fb_to_dict.fb_obj_to_dict(elem)
        for elem in sorted(
            query_pc.query_pcs(proj_uuid, grpc_channel),
            key=lambda pc: pc.Header().UuidMsgs().decode(),
        )
    ]

    default_rot = {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}

    # sort inner bb list and add rotation to the values as this is set with default values by the server
    for bbs in bbs_ls:
        for label in bbs["LabelsBb"]:
            label["BoundingBoxLabeled"].sort(key=lambda bb: bb["LabelWithInstance"]["InstanceUuid"])
            for bb in label["BoundingBoxLabeled"]:
                bb["BoundingBox"]["Rotation"] = default_rot

    for bbs in queried_pcs:
        for label in bbs["LabelsBb"]:
            label["BoundingBoxLabeled"].sort(key=lambda bb: bb["LabelWithInstance"]["InstanceUuid"])

    assert len(queried_pcs) == len(bbs_ls)

    for idx, pc in enumerate(queried_pcs):
        # filter for labels in category laterAddedBB
        pc_labeled_bb_with_category = [elem for elem in pc["LabelsBb"] if elem["Category"] == "laterAddedBB"]
        assert pc["Header"]["UuidMsgs"] == bbs_ls[idx]["Header"]["UuidMsgs"]
        assert pc_labeled_bb_with_category == bbs_ls[idx]["LabelsBb"]
