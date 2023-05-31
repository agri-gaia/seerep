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
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval


def preprocess_data(messages):
    # Preprocess data
    gt_bboxes = []
    pred_bboxes = []
    category_ids = []
    category_map = {'weeds': 1, 'maize': 2}

    # Extract information from data
    for message in messages:
        # Extract label, confidence score, and bounding box information
        label = message['label']
        confidence = message['confidence']
        bbox = message['bbox']

        # Map label to category ID
        category_id = category_map[label]

        # Add information to data structures
        gt_bboxes.append(bbox)
        pred_bboxes.append(bbox)
        category_ids.append(category_id)


def evaluate_with_pycocotools4(gt_messages, pred_messages, category_map):
    # initialize COCO objects for ground truth and predictions
    cocoGt = COCO()
    cocoDt = COCO()

    # set up ground truth annotations
    gt_annotations = []
    cocoGt.dataset["categories"] = [
        {"id": category_id, "name": category_name} for category_name, category_id in category_map.items()
    ]

    for message in gt_messages:
        # Extract information from message
        uuidmsg = message['uuidmsg']
        label = message['label']
        bbox = message['bbox']
        image_id = message['image_id']
        image_width = message['image_width']
        image_height = message['image_height']

        # Map label to category ID
        category_id = category_map[label]

        gt_annotation = {
            'id': hash(uuidmsg),
            'image_id': image_id,
            'category_id': category_id,
            'bbox': bbox,
            'area': bbox[2] * bbox[3],
            'iscrowd': 0,
        }
        gt_annotations.append(gt_annotation)

        # Add image info
        image_info = {"id": image_id, "width": image_width, "height": image_height}
        if "images" not in cocoGt.dataset:
            cocoGt.dataset["images"] = []
        cocoGt.dataset["images"].append(image_info)
    cocoGt.dataset['annotations'] = gt_annotations

    # set up prediction annotations
    dt_annotations = []
    for message in pred_messages:
        # Extract information from message
        uuidmsg = message['uuidmsg']
        label = message['label']
        confidence = message['confidence']
        bbox = message['bbox']
        image_id = message['image_id']
        image_width = message['image_width']
        image_height = message['image_height']

        # Map label to category ID
        category_id = category_map[label]

        dt_annotation = {
            'id': hash(uuidmsg),
            'image_id': image_id,
            'category_id': category_id,
            'bbox': bbox,
            'score': confidence,
            'images': 1,
        }
        dt_annotations.append(dt_annotation)

        # Add image info
        image_info = {"id": image_id, "width": image_width, "height": image_height}
        if "images" not in cocoDt.dataset:
            cocoDt.dataset["images"] = []
        cocoDt.dataset["images"].append(image_info)
    cocoDt.dataset['annotations'] = dt_annotations
    cocoDt.dataset["categories"] = [
        {"id": category_id, "name": category_name} for category_name, category_id in category_map.items()
    ]

    # load predictions into cocoDt

    # create indices
    cocoGt.createIndex()
    cocoDt.createIndex()
    cocoDt.loadRes(dt_annotations)

    # evaluate
    cocoEval = COCOeval(cocoGt, cocoDt, iouType='bbox')
    cocoEval.evaluate()
    cocoEval.accumulate()
    cocoEval.summarize()

    # extract metrics
    iou = cocoEval.stats[0] if len(cocoEval.stats) > 0 else 0
    ap = cocoEval.stats[1] if len(cocoEval.stats) > 1 else 0

    return iou, ap


def test_evaluate_with_pycocotools4():
    gt_messages = [
        {
            'uuidmsg': '63da9961-f964-415b-8463-133fd6a444a7',
            'label': 'weeds',
            'bbox': (738.3054809570312, 855.8366088867188, 894.767822265625, 959.7492370605469),
            'image_id': 1,
            'image_width': 640,
            'image_height': 480,
        },
        {
            'uuidmsg': '77c01194-d6a2-461e-943f-16ba45dc55c0',
            'label': 'weeds',
            'bbox': (620.421142578125, 828.3427734375, 849.9344482421875, 939.3506469726562),
            'image_id': 1,
            'image_width': 640,
            'image_height': 480,
        },
        {
            'uuidmsg': '821873f7-3e49-4173-8256-5641abbc7fa1',
            'label': 'maize',
            'bbox': (693.8901977539062, 556.0573425292969, 1196.736083984375, 948.911865234375),
            'image_id': 1,
            'image_width': 640,
            'image_height': 480,
        },
    ]

    pred_messages = [
        {
            'uuidmsg': '63da9961-f964-415b-8463-133fd6a444a7',
            'label': 'weeds',
            'confidence': 0.9566268920898438,
            'bbox': (738.3054809570312, 855.8366088867188, 894.767822265625, 959.7492370605469),
            'image_id': 1,
            'image_width': 640,
            'image_height': 480,
        },
        {
            'uuidmsg': '77c01194-d6a2-461e-943f-16ba45dc55c0',
            'label': 'weeds',
            'confidence': 0.9064179062843323,
            'bbox': (620.421142578125, 828.3427734375, 849.9344482421875, 939.3506469726562),
            'image_id': 1,
            'image_width': 640,
            'image_height': 480,
        },
        {
            'uuidmsg': '821873f7-3e49-4173-8256-5641abbc7fa1',
            'label': 'maize',
            'confidence': 0.894409716129303,
            'bbox': (693.8901977539062, 556.0573425292969, 1196.736083984375, 948.911865234375),
            'image_id': 1,
            'image_width': 640,
            'image_height': 480,
        },
    ]
    category_map = {'weeds': 1, 'maize': 2}

    iou, ap = evaluate_with_pycocotools4(gt_messages, pred_messages, category_map)

    # Check that the output is as expected
    assert iou == 1.0
    assert ap == 1.0


def evaluate_with_pycocotools3(gt_bboxes, pred_bboxes, category_ids, confidence_score=0.6):
    # initialize COCO objects for ground truth and predictions
    cocoGt = COCO()
    cocoDt = COCO()
    # set up ground truth annotations
    # image_info = {"id": 1, "width": 640, "height": 480}
    # cocoGt.dataset["image"] = [image_info]
    gt_annotations = []
    image_info = {"id": 1, "width": 640, "height": 480}
    cocoGt.dataset["images"] = [image_info]
    lable = [{"id": 1, "name": "object"}]
    cocoGt.dataset["categories"] = lable
    for i, bbox in enumerate(gt_bboxes):
        gt_annotation = {
            'id': i + 1,
            'image_id': 1,
            'category_id': category_ids[i],
            'bbox': bbox,
            'area': bbox[2] * bbox[3],
            'iscrowd': 0,
        }
        gt_annotations.append(gt_annotation)
    cocoGt.dataset['annotations'] = gt_annotations

    # set up prediction annotations
    dt_annotations = []
    for i, bbox in enumerate(pred_bboxes):
        dt_annotation = {
            'id': i + 1,
            'image_id': 1,
            'category_id': category_ids[i],
            'bbox': bbox,
            'score': confidence_score,
            'images': 1,
        }
        dt_annotations.append(dt_annotation)
    cocoDt.dataset['annotations'] = dt_annotations
    cocoDt.dataset["images"] = [image_info]
    cocoDt.dataset["categories"] = lable

    # load predictions into cocoDt

    # create indices
    cocoGt.createIndex()
    cocoDt.createIndex()
    cocoDt = cocoDt.loadRes(dt_annotations)

    # evaluate
    cocoEval = COCOeval(cocoGt, cocoDt, iouType='bbox')
    cocoEval.evaluate()
    cocoEval.accumulate()
    cocoEval.summarize()

    # extract metrics
    iou = cocoEval.stats[0] if len(cocoEval.stats) > 0 else 0
    ap = cocoEval.stats[1] if len(cocoEval.stats) > 1 else 0

    return iou, ap


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


def xywh_to_xyxy(bbox):
    x, y, w, h = bbox
    return [x - w / 2, y - h / 2, x + w / 2, y + h / 2]


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
    pred_bboxes2 = np.array([[50, 50, 150, 150], [100, 100, 200, 200], [50, 50, 150, 150], [0, 0, 50, 50]])

    # Ground truth boxes in format [x1, y1, x2, y2]
    gt_bboxes2 = np.array([[50, 50, 150, 150], [100, 100, 200, 200], [0, 0, 100, 100], [75, 75, 125, 125]])
    gt_bboxes2[:, 2:] -= gt_bboxes[:, :2]
    pred_bboxes2[:, 2:] -= pred_bboxes[:, :2]

    # iou, ap = evaluate_with_pycocotools(gt_bboxes2, pred_bboxes2)

    test_pred = np.array([603.821, 784.311, 1076.294, 1135.188])
    test_gt = np.array([510.943, 696.685, 978.965, 1182.016])
    test_pred = xywh_to_xyxy(test_pred)
    test_gt = xywh_to_xyxy(test_gt)
    print(test_pred)
    print(test_gt)
    print(test_gt)
    iou, ap = evaluate_with_pycocotools([[436.973, 532.643, 773.648, 935.734]], [[271.446, 507.357, 744.418, 882.351]])
    print("Average Precesion", ap)
    print("intersection over union", iou)
    # iou2, ap2 = evaluate_with_pycocotools3(gt_bboxes2, pred_bboxes2, category_ids=[1, 1, 1, 1], confidence_score=0.6)
# print("Average Precesion22", ap2),
# print("intersection over union22", iou2)

# print("evaaaaaaaaaaaaaa", test_evaluate_with_pycocotools4())
