# NOTE: I think moving this into a seperate repository would be a good idea.
import json
import time
from pathlib import Path
from typing import Generator, List

import cv2
import flatbuffers
from seerep.fb import Image, Label, LabelCategory
from seerep.fb import image_service_grpc_fb as ImageService
from seerep.util.common import get_gRPC_channel

# TODO: the imports should be different
from seerep.util.fb_helper import create_image as create_fb_image
from seerep.util.fb_helper import create_label as create_fb_label
from seerep.util.fb_helper import create_label_category as create_fb_label_category
from seerep.util.fb_helper import createHeader as create_fb_header
from seerep.util.fb_helper import createTimeStamp as create_fb_timestamp

DATUMARO_JSON_PATH = "/seerep/seerep-data/datumaro_test/annotations/default.json"
IMAGE_FOLDER_PATH = "/seerep/seerep-data/datumaro_test/images/default/"
FRAME_ID = "map"
# NOTE: Mock data
CAMERA_INTRINSIC_UUID = "7c8c6037-93a4-4b78-a5d3-c62d4b21fa57"
PROJECT_UUID = "ce9dbda6-63f3-4c6d-81b3-41f39f031647"


class DatumaroImporter:
    def __init__(self, datumaro_path: str, image_folder: str):
        self.datumaro_path = Path(datumaro_path)
        if not self.valid_file(self.datumaro_path, "json"):
            raise ValueError('Invalid datumaro JSON path')
        self.image_folder = Path(image_folder)
        if not self.valid_folder(self.image_folder):
            raise ValueError('Invalid image folder path')
        self.labels = None

    def valid_folder(self, path: Path) -> bool:
        return path.exists() and path.is_dir()

    def valid_file(self, path: Path, extension: str) -> bool:
        return path.exists() and path.is_file() and path.suffix == "." + extension

    def create_labels(self, labels: dict) -> None:
        self.labels = {item['name']: {k: v for k, v in item.items() if k != 'name'} for item in labels}

    def create_label_categories(
        self, labels: List[Label.Label], annotations: str, category: str = "CVAT"
    ) -> LabelCategory.LabelCategory:
        return create_fb_label_category(flatbuffers.Builder(1024), labels, annotations, category)

    # NOTE: currently only works with pngs
    def create_image(self, fbb, image_info: dict, category_labels: List[LabelCategory.LabelCategory]) -> Image.Image:
        image_path = self.image_folder.joinpath(Path(image_info['path']))
        if not self.valid_file(image_path, "png"):
            raise ValueError('Invalid image path')
        cv_image = cv2.imread(str(image_path))

        fb_timestamp = create_fb_timestamp(fbb, int(time.time()))
        fb_header = create_fb_header(fbb, fb_timestamp, FRAME_ID, PROJECT_UUID)
        # TODO: add labels
        fb_image = create_fb_image(fbb, fb_header, "png", False, cv_image, CAMERA_INTRINSIC_UUID, category_labels)
        fbb.Finish(fb_image)
        return fbb.Output()

    def iterate_json(self, data: dict) -> Generator[bytes, None, None]:
        fbb = flatbuffers.Builder(1024)
        for item in data['items']:
            label_ids = set([annotation['label_id'] for annotation in item['annotations']])
            fb_labels = [create_fb_label(fbb, list(self.labels.keys())[label_id], label_id) for label_id in label_ids]
            fb_labels_with_category = self.create_label_categories(fb_labels, str(item))
            yield bytes(self.create_image(fbb, item['image'], fb_labels_with_category))

    def run(self) -> None:
        with open(self.datumaro_path, 'r') as f:
            # TODO: this loads the entire json file into memory, bad
            data = json.load(f)
            self.create_labels(data['categories']['label']['labels'])
            gen = self.iterate_json(data)
            grpc_channel = get_gRPC_channel()
            image_stub = ImageService.ImageServiceStub(grpc_channel)
            try:
                image_stub.TransferImage(gen)
            except Exception as e:
                print(e)


if __name__ == '__main__':
    di = DatumaroImporter(DATUMARO_JSON_PATH, IMAGE_FOLDER_PATH)
    di.run()
