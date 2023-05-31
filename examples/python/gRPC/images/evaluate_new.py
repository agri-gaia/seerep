from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
import json

# from plot_results import plot_precision_recall
from pycocotools import mask as cocomask

# [xmin, ymin, width, height]
from pycocotools import mask as cocomask
import numpy as np

# pycocotools calculates using masks
def compute_iou_pycoco(box1_np, box2_np):
    rles = cocomask.frPyObjects([box1_np.tolist(), box2_np.tolist()], int(box1_np[2]), int(box1_np[3]))
    area_intersection = cocomask.area(cocomask.merge(rles))
    area_union = sum(cocomask.area(rle) for rle in rles)
    iou = area_intersection / area_union if area_union else 0
    return iou


# without pycocotools
def compute_iou(box1, box2):
    x_a = max(box1[0], box2[0])
    y_a = max(box1[1], box2[1])
    x_b = min(box1[0] + box1[2], box2[0] + box2[2])
    y_b = min(box1[1] + box1[3], box2[1] + box2[3])

    inter_area = max(0, x_b - x_a + 1) * max(0, y_b - y_a + 1)

    if inter_area == 0:
        return 0

    box1_area = box1[2] * box1[3]
    box2_area = box2[2] * box2[3]

    iou = inter_area / float(box1_area + box2_area - inter_area)

    return iou


def evaluate_predictions(gt_file_path, pred_file_path):
    # load thr ground truth and predictions data
    with open(gt_file_path, "r") as f:
        gt_data = json.load(f)

    with open(pred_file_path, "r") as f:
        pred_data = json.load(f)

    # initialization
    coco_gt = COCO()
    coco_gt.dataset = gt_data
    coco_gt.createIndex()

    coco_pred = COCO()
    coco_pred.dataset = pred_data
    coco_pred.createIndex()

    # evaluate the predictions
    coco_eval = COCOeval(coco_gt, coco_pred, "bbox")

    coco_eval.evaluate()
    coco_eval.accumulate()
    coco_eval.summarize()

    return coco_eval
