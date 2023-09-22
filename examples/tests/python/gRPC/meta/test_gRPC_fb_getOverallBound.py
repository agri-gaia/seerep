# test file
#  gRPC_fb_getOverallBound.py
from typing import List, Set, Tuple

import flatbuffers
from gRPC.images import gRPC_pb_sendLabeledImageGrid as send_grid
from gRPC.meta import gRPC_fb_getOverallBound
from gRPC.tf import gRPC_fb_getTf as get_tf
from seerep.fb import TransformStamped, Vector3


def test_gRPC_fb_getOverallBound(grpc_channel, project_setup) -> None:
    proj_name, proj_uuid = project_setup

    print(f"Testing project: {proj_name}; {proj_uuid}")

    sent_grid, tf_times, camin = send_grid.send_labeled_image_grid(
        proj_uuid, grpc_channel
    )

    # get send tfs
    tfs: List[TransformStamped.TransformStamped] = get_tf.get_tfs(
        tf_times, proj_uuid, grpc_channel
    )

    sorted_tfs = sorted(tfs, key=lambda tf: tf.Header().Stamp().Seconds())

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

    # sort imgs by timestamp
    imgs_flattened = [
        img for inner_lst in sent_grid for img_lst in inner_lst for _, img in img_lst
    ]
    imgs_sorted = sorted(imgs_flattened, key=lambda img: img.header.stamp.seconds)

    img_coords: List[Tuple] = []
    # find nearest tf for each image according to timestamp

    for img in imgs_sorted:
        start = 0
        end = len(sorted_tfs) - 1
        prev_ts = sorted_tfs[0].Header().Stamp().Seconds()
        curr_ts = sorted_tfs[1].Header().Stamp().Seconds()
        curr_idx = 1
        while True:
            if img.header.stamp.seconds == curr_ts:
                coordinate_trans = sorted_tfs[curr_idx].Transform().Translation()
                img_coords.append((img, coordinate_trans))
                break

            if (
                img.header.stamp.seconds < curr_ts
                and curr_idx - 1 >= 0
                and img.header.stamp.seconds > prev_ts
            ):
                # interpolate coordinates
                time_diff = curr_ts - prev_ts
                weighted_diff_prev = (img.header.stamp.seconds - prev_ts) / time_diff
                weighted_diff_curr = (curr_ts - img.header.stamp.seconds) / time_diff
                w_x = (
                    weighted_diff_curr
                    * sorted_tfs[curr_idx].Transform().Translation().X()
                    + weighted_diff_prev
                    * sorted_tfs[prev_ts].Transform().Translation().X()
                )
                w_y = (
                    weighted_diff_curr
                    * sorted_tfs[curr_idx].Transform().Translation().Y()
                    + weighted_diff_prev
                    * sorted_tfs[prev_ts].Transform().Translation().X()
                )
                w_z = (
                    weighted_diff_curr
                    * sorted_tfs[curr_idx].Transform().Translation().Z()
                    + weighted_diff_prev
                    * sorted_tfs[prev_ts].Transform().Translation().Z()
                )

                builder = flatbuffers.Builder(1024)

                Vector3.Start(builder)
                Vector3.AddX(builder, w_x)
                Vector3.AddY(builder, w_y)
                Vector3.AddZ(builder, w_z)
                coordinate_trans = Vector3.End(builder)

                img_coords.append((img, coordinate_trans))
                break

            if end < start or prev_ts is None:
                img_coords.append((img, None))
                break

            # else search next tf stamp via binary search
            elif img.header.stamp.seconds < curr_ts:
                end = curr_idx - 1
                curr_idx = (start + end) // 2
                curr_ts = sorted_tfs[curr_idx].Header().Stamp().Seconds()
                if curr_idx - 1 >= 0:
                    prev_ts = sorted_tfs[curr_idx - 1].Header().Stamp().Seconds()
                else:
                    prev_ts = None

            elif img.header.stamp.seconds > curr_ts:
                start = curr_idx + 1
                curr_idx = (start + end) // 2
                curr_ts = sorted_tfs[curr_idx].Header().Stamp().Seconds()
                prev_ts = sorted_tfs[curr_idx - 1].Header().Stamp().Seconds()

    coords = [coord for _, coord in img_coords]

    x_coords = set([c.X() for c in coords])
    y_coords = set([c.Y() for c in coords])
    z_coords = set([c.Z() for c in coords])

    # calculate center point and spatial extent based on the coordinates
    center_point_x = (max(x_coords) - min(x_coords)) / 2
    center_point_y = (max(y_coords) - min(y_coords)) / 2
    center_point_z = (max(z_coords) - min(z_coords)) / 2

    spatial_extent_x = max(x_coords) - min(x_coords)
    spatial_extent_y = max(y_coords) - min(y_coords)
    spatial_extent_z = max(z_coords) - min(z_coords)

    # find min time in sets
    min_time = min(timestamps)
    max_time = max(timestamps)

    # request metadata
    (
        resp_overall_time_intervall,
        resp_overall_bb,
        resp_all_categories,
        resp_all_labels,
    ) = gRPC_fb_getOverallBound.get_metadata(proj_uuid, grpc_channel)

    # check if any of the values of the set contain only zero
    # cause then the floating point numbers of resp_overall_bb are not exactly zero
    epsilon = 1e-34

    # for the x coordinate
    if x_coords == {0.0}:
        assert abs(resp_overall_bb[0][0] - center_point_x) < epsilon
        assert abs(resp_overall_bb[1][0] - spatial_extent_x) < epsilon
        resp_overall_bb[0][0] = 0.0
        resp_overall_bb[1][0] = 0.0

    if y_coords == {0.0}:
        assert abs(resp_overall_bb[0][1] - center_point_y) < epsilon
        assert abs(resp_overall_bb[1][1] - spatial_extent_y) < epsilon
        resp_overall_bb[0][1] = 0.0
        resp_overall_bb[1][1] = 0.0

    if z_coords == {0.0}:
        assert abs(resp_overall_bb[0][2] - center_point_z) < epsilon
        assert abs(resp_overall_bb[1][2] - spatial_extent_z) < epsilon
        resp_overall_bb[0][2] = 0.0
        resp_overall_bb[1][2] = 0.0

    # check if the values are the same
    assert resp_overall_time_intervall[0][0] == min_time
    assert resp_overall_time_intervall[1][0] == max_time

    assert resp_overall_bb[0][0] == center_point_x
    assert resp_overall_bb[0][1] == center_point_y
    assert resp_overall_bb[0][2] == center_point_z

    assert resp_overall_bb[1][0] == spatial_extent_x
    assert resp_overall_bb[1][1] == spatial_extent_y
    assert resp_overall_bb[1][2] == spatial_extent_z

    assert set(resp_all_categories) == categories
    assert set(resp_all_labels) == labels
