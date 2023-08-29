# test file for
#  gRPC_pb_getOverallBound.py
# requires:
#  gRPC_sendLabeledImageGrid.py
from typing import Set, Tuple

from gRPC.images import gRPC_pb_sendLabeledImageGrid as send_grid
from gRPC.meta import gRPC_pb_getOverallBound


def test_gRPC_pb_getOverallBound(grpc_channel, project_setup) -> None:
    proj_name, proj_uuid = project_setup

    print(f"Testing project: {proj_name}; {proj_uuid}")

    sent_grid = send_grid.send_labeled_image_grid(proj_uuid, grpc_channel)

    # get information such as timestamp, boundingbox, category, all labels of images in a specified project
    timestamps: Set[int] = set()
    center_points: Set[Tuple[float, float]] = set()
    spatial_extents: Set[Tuple[float, float]] = set()
    categories: Set[str] = set()
    labels: Set[str] = set()

    for inner_lst in sent_grid:
        for img_lst in inner_lst:
            for _, img in img_lst:
                timestamps.add(img.header.stamp.seconds)
                for label_general in img.labels_general:
                    for label_with_instance in label_general.labelWithInstance:
                        labels.add(label_with_instance.label.label)

                for bb_cat in img.labels_bb:
                    categories.add(bb_cat.category)
                    for labelbb in bb_cat.boundingBox2DLabeled:
                        center_points.add(
                            (
                                labelbb.boundingBox.center_point.x,
                                labelbb.boundingBox.center_point.y,
                            )
                        )
                        spatial_extents.add(
                            (
                                labelbb.boundingBox.spatial_extent.x,
                                labelbb.boundingBox.spatial_extent.y,
                            )
                        )
                        labels.add(labelbb.labelWithInstance.label.label)

    # find min time in sets
    min_time = min(timestamps)
    max_time = max(timestamps)

    print(spatial_extents)
    print(center_points)

    # TODO: correctly calculate center point and spatial extent from bounding boxes to an overall bounding box
    # is the bounding box based upon integer coordinates only?
    # the following is incorrect!
    # center_x_overall = sum(center_points_x) / len(center_points_x)
    # center_y_overall = sum(center_points_y) / len(center_points_y)
    # spatial_extent_x_max = max(spatial_extents_x)
    # spatial_extent_y_max = max(spatial_extents_y)
    # print(center_points_x)

    # print(
    #     center_x_overall, center_y_overall, spatial_extent_x_max, spatial_extent_y_max
    # )

    # retrieve the values received by the example
    (
        resp_overall_time_intervall,
        resp_overall_bb,
        resp_all_categories,
        resp_all_labels,
    ) = gRPC_pb_getOverallBound.get_metadata(proj_uuid, grpc_channel)

    print(resp_overall_bb)

    assert resp_overall_time_intervall.time_min.seconds == min_time
    assert resp_overall_time_intervall.time_max.seconds == max_time

    assert set(resp_all_categories.categories) == categories
    assert set(resp_all_labels.labels) == labels
