#!/usr/bin/env python3

import csv
import os
import uuid

import flatbuffers
import imageio.v2 as imageio
import yaml
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createBoundingBox2DLabeledWithCategory,
    createBoundingBoxes2d,
    createBoundingBoxes2dLabeled,
    createHeader,
    createLabelsWithInstance,
    createLabelWithCategory,
    createPoint2d,
    createTimeStamp,
    getOrCreateProject,
)


class yoloAnnotatedImageLoader:
    def __init__(self) -> None:
        self.LABEL_GENERAL = "label.yaml"
        self.IMAGE_ENCODING = "rgb8"
        self.PROJECT_NAME = "aitf-triton-data"
        self.labelCategory = "ground_truth"
        builder = flatbuffers.Builder(1024)
        self.channel = get_gRPC_channel()
        self.projectUuid = getOrCreateProject(builder, self.channel, self.PROJECT_NAME)

        self.time = 1680705904
        self.rootFolder = "/seerep/seerep-data/ai-tf-triton-paper-data"

        self.stubImage = imageService.ImageServiceStub(self.channel)
        self.stubImage.TransferImage(self.__loadImages())

    def __loadImages(self):
        for root, dirs, files in os.walk(self.rootFolder):
            if len(files) > 0:
                labelGeneralPath = os.path.join(root, self.LABEL_GENERAL)
                for file in files:
                    if file.endswith(".png"):
                        imagePath = os.path.join(root, file)
                        labelPath = os.path.join(root, file[:-4] + ".txt")

                        builder = flatbuffers.Builder(1024)
                        imageMsg = self.__createImageMsg(builder, imagePath, labelPath, labelGeneralPath)

                        builder.Finish(imageMsg)
                        yield bytes(builder.Output())

    def __createImageMsg(self, builder, imagePath, labelPath, labelGeneralPath):

        labelGeneral = self.__readLabelGeneral(labelGeneralPath)
        labelGeneral.append(imagePath)
        print(labelGeneral)
        labels = self.__readLabel(labelPath)
        print(labels)
        img = imageio.imread(imagePath)
        imData = builder.CreateByteVector(img.tobytes())

        if len(labels) > 0:
            centerPoints = []
            spatialExtents = []
            labelStrings = []
            for label in labels:
                labelStrings.append('person')
                centerPoints.append(createPoint2d(builder, float(label[1]), float(label[2])))
                spatialExtents.append(createPoint2d(builder, float(label[3]), float(label[4])))

            boundingBoxes = createBoundingBoxes2d(builder, centerPoints, spatialExtents)
            labelWithInstances = createLabelsWithInstance(
                builder,
                labelStrings,
                [1.0 for _ in range(len(labelStrings))],
                [str(uuid.uuid4()) for _ in range(len(labelStrings))],
            )
            labelsBb = createBoundingBoxes2dLabeled(builder, labelWithInstances, boundingBoxes)

            boundingBox2DLabeledWithCategory = createBoundingBox2DLabeledWithCategory(
                builder, builder.CreateString(self.labelCategory), labelsBb
            )

            Image.StartLabelsBbVector(builder, 1)
            builder.PrependUOffsetTRelative(boundingBox2DLabeledWithCategory)
            boundingBox2DLabeledWithCategoryVector = builder.EndVector()

        labelsGeneral = createLabelsWithInstance(
            builder,
            labelGeneral,
            [1.0 for _ in range(len(labelGeneral))],
            [str(uuid.uuid4()) for _ in range(len(labelGeneral))],
        )
        labelsGeneralCat = createLabelWithCategory(builder, [self.labelCategory], [labelsGeneral])

        header = createHeader(
            builder,
            timeStamp=createTimeStamp(builder, self.time, 0.0),
            frame="map",
            projectUuid=self.projectUuid,
        )

        encoding = builder.CreateString(self.IMAGE_ENCODING)
        ci = builder.CreateString("ba3679d0-2e85-40c4-b925-9c483fc4fba0")

        Image.Start(builder)
        Image.AddHeader(builder, header)
        Image.AddData(builder, imData)
        Image.AddEncoding(builder, encoding)
        Image.AddHeight(builder, img.shape[0])
        Image.AddWidth(builder, img.shape[1])
        Image.AddIsBigendian(builder, False)
        if len(labels) > 0:
            Image.AddLabelsBb(builder, boundingBox2DLabeledWithCategoryVector)
        Image.AddLabelsGeneral(builder, labelsGeneralCat)
        Image.AddStep(builder, 3 * img.shape[1])
        Image.AddUuidCameraintrinsics(builder, ci)
        return Image.End(builder)

    def __readLabelGeneral(self, labelGeneralPath):
        with open(labelGeneralPath, "r") as stream:
            try:
                labelGeneralYamlObject = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        labelGeneral = []
        for key in labelGeneralYamlObject['labels']:
            if key != 'sensor':
                labelGeneral.append(key + "_" + labelGeneralYamlObject['labels'][key])
        return labelGeneral

    def __readLabel(self, labelPath):
        label = []
        reader = csv.reader(open(labelPath), delimiter=" ")
        for row in reader:
            label.append(row)
        return label


if __name__ == "__main__":
    yoloAnnotatedImageLoader()
