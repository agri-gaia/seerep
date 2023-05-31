#!/usr/bin/env python3
import os
import sys

import flatbuffers
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
from preprocess_response import (
    extract_data_from_response,
    create_coco_format,
    separate_gt_and_predictions,
)
import json
from evaluate_new import evaluate_predictions, compute_iou
from collections import defaultdict
import numpy as np


# importing util functions. Assuming that these files are in the parent dir

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb

builder = flatbuffers.Builder(1024)
# Default server is localhost !
channel = util.get_gRPC_channel("agrigaia-ur.ni.dfki:9090")

# 1. Get all projects from the server
projectuuid = util_fb.getProject(builder, channel, 'aitf-triton-data')

# 2. Check if the defined project exist; if not exit
if not projectuuid:
    print("project doesn't exist!")
    exit()

# 3. Get gRPC service object
stub = imageService.ImageServiceStub(channel)


# Create all necessary objects for the query
header = util_fb.createHeader(builder, frame="map")
pointMin = util_fb.createPoint(builder, 0.0, 0.0, 0.0)
pointMax = util_fb.createPoint(builder, 100.0, 100.0, 100.0)
boundingboxStamped = util_fb.createBoundingBoxStamped(builder, header, pointMin, pointMax)

timeMin = util_fb.createTimeStamp(builder, 1610549273, 0)
timeMax = util_fb.createTimeStamp(builder, 1938549273, 0)
timeInterval = util_fb.createTimeInterval(builder, timeMin, timeMax)


projectUuids = [builder.CreateString(projectuuid)]
# list of categories
category = ["0"]
# list of labels per category
labels = [
    [  # weather_general_sun
        util_fb.createLabelWithConfidence(builder, "weather_general_sun")
        # util_fb.createLabelWithConfidence(builder, "testlabel0"),
        # util_fb.createLabelWithConfidence(builder, "testlabelgeneral0"),
    ]
]
labelCategory = util_fb.createLabelWithCategory(builder, category, labels)
dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

# 4. Create a query with parameters
# all parameters are optional
# with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
query = util_fb.createQuery(
    builder,
    # boundingBox=boundingboxStamped,
    # timeInterval=timeInterval,
    # labels=labelCategory,
    # mustHaveAllLabels=True,
    projectUuids=projectUuids,
    # instanceUuids=instanceUuids,
    # dataUuids=dataUuids,
    withoutData=True,
)
builder.Finish(query)
buf = builder.Output()

# Query the server for images matching the query and iterate over them, extract the data from the response and save it in a list
output = []
for responseBuf in stub.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)
    data = extract_data_from_response(response)
    output.append(data)
# print(f"data from seerep is: {output[19]}")
coco_format_data = create_coco_format(output)

# save the coco format data to a json file
file_path = "examples/python/gRPC/images/output_coco_format5.json"
with open(file_path, "w") as f:
    json.dump(coco_format_data, f)
separate_gt_and_predictions(
    "examples/python/gRPC/images/output_coco_format5.json",
    "examples/python/gRPC/images/ground_truth5.json",
    "examples/python/gRPC/images/predictions5.json",
)
# coco general evaluation
coco_eval = evaluate_predictions(
    "examples/python/gRPC/images/ground_truth5.json", "examples/python/gRPC/images/predictions5.json"
)
# load ground truth and predictions
with open("examples/python/gRPC/images/ground_truth5.json") as f:
    ground_truth = json.load(f)
with open("examples/python/gRPC/images/predictions5.json") as f:
    predictions = json.load(f)

# index annotations by image_id to calculate individual iou
ground_truth_by_image = defaultdict(list)
predictions_by_image = defaultdict(list)
for ann in ground_truth['annotations']:
    ground_truth_by_image[ann['image_id']].append(ann['bbox'])
for ann in predictions['annotations']:
    predictions_by_image[ann['image_id']].append(ann['bbox'])


iou_values = {}

for image_id in ground_truth_by_image:
    gt_boxes = ground_truth_by_image[image_id]
    pred_boxes = predictions_by_image[image_id]
    iou_list = []
    for gt_box in gt_boxes:
        for pred_box in pred_boxes:
            if len(gt_box) >= 4 and len(pred_box) >= 4:
                gt_box_np = np.array(gt_box)
                pred_box_np = np.array(pred_box)

                try:
                    iou = compute_iou(gt_box_np, pred_box_np)
                    print(
                        f"Image {image_id} - IoU between ground truth box {gt_box} and predicted box {pred_box} is {iou}"
                    )
                    iou_list.append(iou)
                except IndexError:
                    print(f"Error computing IoU for Image {image_id} - GT Box: {gt_box}, Predicted Box: {pred_box}")
            else:
                print(f"Image {image_id} - Bounding box has less than four elements: GT {gt_box}, Pred {pred_box}")

    iou_values[image_id] = iou_list
# print(f"iou values calculated: {len(iou_values)}")

# save the iou values
with open("examples/python/gRPC/images/iou_values.json", "w") as f:
    json.dump(iou_values, f)
