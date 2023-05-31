import json
import re
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
import uuid


def uuid_to_int(uuid_str):
    return uuid.UUID(uuid_str).int


def extract_data_from_response(response):
    uuidmsg = response.Header().UuidMsgs().decode('utf-8')
    num_labels = response.LabelsBbLength()
    # print(f"number of lables is {num_labels}")

    yolo_boxes = []
    ground_truth_boxes = []
    if response.LabelsBbLength() > 1:

        # Iterate through all the labels and store bounding boxes in their respective lists
        for i in range(num_labels):
            category = response.LabelsBb(i).Category().decode('utf-8')
            label = response.LabelsBb(i).BoundingBox2dLabeled(0).LabelWithInstance().Label().Label().decode("utf-8")
            confidence = response.LabelsBb(i).BoundingBox2dLabeled(0).LabelWithInstance().Label().Confidence()

            bbox = (
                str(response.LabelsBb(i).BoundingBox2dLabeled(0).BoundingBox().CenterPoint().X()),
                str(response.LabelsBb(i).BoundingBox2dLabeled(0).BoundingBox().CenterPoint().Y()),
                str(response.LabelsBb(i).BoundingBox2dLabeled(0).BoundingBox().SpatialExtent().X()),
                str(response.LabelsBb(i).BoundingBox2dLabeled(0).BoundingBox().SpatialExtent().Y()),
            )

            box_data = {
                "label": label,
                "confidence": confidence,
                "bbox": bbox,
            }

            if category == "ground_truth":
                ground_truth_boxes.append(box_data)
            elif category == "YOLOv5nCOCO":
                yolo_boxes.append(box_data)
            # might need other categories

    data = {
        "uuidmsg": uuidmsg,
        "num_labels": num_labels,
        "yolo_boxes": yolo_boxes,
        "ground_truth_boxes": ground_truth_boxes,
    }

    return data


def denormalize(value, size):
    return float(value) * size


def create_coco_format(data_list):
    annotations = []
    images = []
    ann_id = 0

    for data in data_list:
        image_id = uuid_to_int(data["uuidmsg"])

        # image metadata
        images.append(
            {
                "id": image_id,
                "width": 1028,
                "height": 720,
                "file_name": f"{data['uuidmsg']}.jpg",
            }
        )

        for box_data in data["ground_truth_boxes"]:
            x_center, y_center, x_extent, y_extent = box_data['bbox']
            # denormalize the coordinates for the groundtruth
            x_center = denormalize(x_center, 1280)
            y_center = denormalize(y_center, 720)
            x_extent = denormalize(x_extent, 1280)
            y_extent = denormalize(y_extent, 720)
            x_min = x_center - (x_extent / 2)
            y_min = y_center - (y_extent / 2)
            width = x_extent
            height = y_extent

            ann_id += 1
            annotations.append(
                {
                    "id": ann_id,
                    "image_id": image_id,
                    "category_id": 1,  # assuming all objects are of the same category
                    "segmentation": [],
                    "area": 0.0,
                    "bbox": [x_min, y_min, width, height],
                    "iscrowd": 0,
                    "score": box_data["confidence"],
                    "type": "ground_truth",
                }
            )
        # YOLOv5 predictions
        for box_data in data["yolo_boxes"]:
            x_center, y_center, x_extent, y_extent = box_data['bbox']
            x_min = float(x_center) - (float(x_extent) / 2)
            y_min = float(y_center) - (float(y_extent) / 2)
            width = float(x_extent)
            height = float(y_extent)
            ann_id += 1
            annotations.append(
                {
                    "id": ann_id,
                    "image_id": image_id,
                    "category_id": 1,
                    "segmentation": [],
                    "area": 0.0,
                    "bbox": [x_min, y_min, width, height],
                    "iscrowd": 0,
                    "score": box_data["confidence"],
                    "type": "YOLOv5",
                }
            )

    categories = [{"id": 1, "name": "person"}]

    coco_format_data = {
        "annotations": annotations,
        "images": images,
        "categories": categories,
    }

    return coco_format_data


def separate_gt_and_predictions(combined_json_path, gt_output_path, pred_output_path):
    # load the combined JSON file
    with open(combined_json_path, "r") as f:
        data = json.load(f)

    # separate ground truth and YOLO predictions
    gt_data = {"images": data["images"], "categories": data["categories"], "annotations": []}
    pred_data = {"images": data["images"], "categories": data["categories"], "annotations": []}

    gt_count = 0
    pred_count = 0
    ignored_count = 0

    for ann in data["annotations"]:
        if "type" in ann:
            if ann["type"] == "ground_truth":
                gt_data["annotations"].append(ann)
                gt_count += 1
            elif ann["type"] == "YOLOv5":
                pred_data["annotations"].append(ann)
                pred_count += 1

    # count ignored predictions
    image_ids_with_gt = set([gt_ann["image_id"] for gt_ann in gt_data["annotations"]])
    ignored_count = sum([1 for pred_ann in pred_data["annotations"] if pred_ann["image_id"] not in image_ids_with_gt])

    # remove ignored predictions
    pred_data["annotations"] = [
        pred_ann for pred_ann in pred_data["annotations"] if pred_ann["image_id"] in image_ids_with_gt
    ]

    # Save the separated ground truth and predictions JSON files
    with open(gt_output_path, "w") as f:
        json.dump(gt_data, f)

    with open(pred_output_path, "w") as f:
        json.dump(pred_data, f)

    print(f"Ground truth count: {gt_count}")
    print(f"Predictions count: {pred_count}")
    print(f"Ignored predictions count: {ignored_count}")
