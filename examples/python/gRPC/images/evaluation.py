import numpy as np
from pycocotools import mask
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval


# confidence is called score in coco format
# for later
# annotations = []
# for i in range(num_bboxes):
#     annotation = {
#         "id": i,
#         "image_id": <image_id>,
#         "category_id": <category_id>,
#         "bbox": <bbox_coords>,
#         "area": <bbox_area>,
#         "iscrowd": 0,
#         "segmentation": [],
#         "score": <confidence_score>
#     }
#     annotations.append(annotation)


def evaluate_with_pycocotools(gt_bboxes, pred_bboxes, lable=[{"id": 1, "name": "object"}], confidence_score=0.6):
    # initialize
    cocoGt = COCO()
    # add 'annotations' key to the dataset dictionary of cocoGt
    cocoGt.dataset["annotations"] = []
    cocoGt.dataset["categories"] = lable  # we need the class id here from seerep(maybe the response lable asl Marc)
    image_info = {"id": 1, "width": 640, "height": 480}
    cocoGt.dataset["images"] = [image_info]
    ann_id = 1
    # gt_anns=[]
    # for the future we can use cocoGt.loadAnns(gt_bboxes) if the ground truth annotations are already stored in a file in the COCO format
    for bbox in gt_bboxes:
        ann = {
            "id": ann_id,
            "image_id": 1,
            "category_id": 1,
            "bbox": bbox,
            "area": (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]),
            "iscrowd": 0,
        }
        cocoGt.dataset["annotations"].append(ann)
        # print("GT annotations", cocoGt.dataset['annotations'])
        ann_id += 1
        # gt_anns.append(ann)
    cocoGt.createIndex()

    # initialize COCO detections api
    cocoDt = cocoGt.loadRes(cocoGt.dataset["annotations"])

    # add prediction bounding boxes to COCO detections api
    pred_anns = []

    for bbox in pred_bboxes:
        ann = {
            "image_id": 1,
            "id": ann_id,
            "category_id": 1,
            "bbox": bbox,
            "score": confidence_score,
        }
        cocoDt.dataset["annotations"].append(ann)
        pred_anns.append(ann)
    cocoDt.createIndex()

    cocoDt_pred = cocoGt.loadRes(pred_anns)

    # run evaluation
    # cocoEval = COCOeval(cocoGt, 'bbox')

    cocoEval = COCOeval(cocoGt, cocoDt_pred, "bbox")
    cocoEval.evaluate()
    cocoEval.accumulate()
    cocoEval.summarize()
    iou = cocoEval.stats[0]  # if len(cocoEval.stats) > 0 else 0
    ap = cocoEval.stats[1]  # if len(cocoEval.stats) > 1 else 0
    return iou, ap


if __name__ == "__main__":
    # define the ground truth and prediction bounding boxes

    gt_bboxes = np.array(
        [
            [100, 100, 200, 200],
            [300, 300, 400, 400],
            [500, 500, 600, 600],
            [700, 700, 800, 800],
        ]
    )
    pred_bboxes = np.array(
        [
            [90, 90, 210, 210],
            [310, 310, 390, 390],
            [520, 520, 580, 580],
            [690, 690, 810, 810],
        ]
    )
    iou, ap = evaluate_with_pycocotools(gt_bboxes, pred_bboxes)
    print("Average Precesion", ap)
    print("intersection over union", iou)
