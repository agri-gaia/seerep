# test file for
#   gRPC_fb_addLabel.py (Pointcloud)
# requires:
#   gRPC_fb_sendPointCloud.py
#   gRPC_fb_queryPointCloud.py
from gRPC.pointcloud import gRPC_fb_addLabel as add_label
from gRPC.pointcloud import gRPC_fb_queryPointCloud as query_pc
from gRPC.pointcloud import gRPC_fb_sendPointCloud as send_pc
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def test_addLabelToPCs(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    # send pointclouds to the project
    send_pc.send_pointcloud(proj_uuid, grpc_channel)

    sent_label = add_label.add_pc_label_raw(proj_uuid, grpc_channel)

    assert sent_label is not None

    queried_pcs = [
        fb_flatc_dict(pcl, SchemaFileNames.POINT_CLOUD_2)
        for pcl in query_pc.query_pcs_raw(proj_uuid, grpc_channel)
    ]

    for label_pc_uuid, label_pc in sent_label:
        pc_label = [
            pc
            for pc in queried_pcs
            if pc["header"]["uuid_msgs"] == label_pc_uuid
        ][0]

        # iterate through all categories of the image
        filtered_label = [
            label
            for label in pc_label["labels"]
            if label["category"] == "laterAddedLabel"
        ]

        sent_labels = fb_flatc_dict(
            label_pc, SchemaFileNames.DATASET_UUID_LABEL
        )["labels"]

        assert len(filtered_label) == len(sent_labels)

        for label_cat in filtered_label:
            assert label_cat in sent_labels
