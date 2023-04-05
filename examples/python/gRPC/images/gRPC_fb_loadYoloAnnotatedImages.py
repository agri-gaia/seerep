#!/usr/bin/env python3

import csv
import os
import sys
import uuid

import flatbuffers
import imageio.v2 as imageio
import numpy as np
import yaml
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import util
import util_fb


class yoloAnnotatedImageLoader:
    def __init__(self) -> None:
        self.LABEL_GENERAL = "label.yaml"
        self.IMAGE_ENCODING = "rgb8"
        self.PROJECT_NAME = "aitf-triton-data"
        self.labelCategory = "ground_truth"
        self.builder = flatbuffers.Builder(1024)
        self.channel = util.get_gRPC_channel()
        self.projectUuid = util_fb.getOrCreateProject(self.builder, self.channel, self.PROJECT_NAME)

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

                        imageMsg = self.__createImageMsg(imagePath, labelPath, labelGeneralPath)

                        self.builder.Finish(imageMsg)
                        yield bytes(self.builder.Output())

    def __createImageMsg(self, imagePath, labelPath, labelGeneralPath):

        labelGeneral = self.__readLabelGeneral(labelGeneralPath)
        labelGeneral.append(imagePath)
        print(labelGeneral)
        labels = self.__readLabel(labelPath)
        print(labels)
        img = imageio.imread(imagePath)
        imData = self.builder.CreateByteVector(img.tobytes())

        if len(labels) > 0:
            centerPoints = []
            spatialExtents = []
            labelStrings = []
            for label in labels:
                labelStrings.append('person')
                centerPoints.append(util_fb.createPoint2d(self.builder, float(label[1]), float(label[2])))
                spatialExtents.append(util_fb.createPoint2d(self.builder, float(label[3]), float(label[4])))

            boundingBoxes = util_fb.createBoundingBoxes2d(self.builder, centerPoints, spatialExtents)
            labelWithInstances = util_fb.createLabelsWithInstance(
                self.builder,
                labelStrings,
                [1.0 for _ in range(len(labelStrings))],
                [str(uuid.uuid4()) for _ in range(len(labelStrings))],
            )
            labelsBb = util_fb.createBoundingBoxes2dLabeled(self.builder, labelWithInstances, boundingBoxes)

            boundingBox2DLabeledWithCategory = util_fb.createBoundingBox2DLabeledWithCategory(
                self.builder, self.builder.CreateString(self.labelCategory), labelsBb
            )

            Image.StartLabelsBbVector(self.builder, 1)
            self.builder.PrependUOffsetTRelative(boundingBox2DLabeledWithCategory)
            boundingBox2DLabeledWithCategoryVector = self.builder.EndVector()

        labelsGeneral = util_fb.createLabelsWithInstance(
            self.builder,
            labelGeneral,
            [1.0 for _ in range(len(labelGeneral))],
            [str(uuid.uuid4()) for _ in range(len(labelGeneral))],
        )
        labelsGeneralCat = util_fb.createLabelWithCategory(self.builder, [self.labelCategory], [labelsGeneral])

        header = util_fb.createHeader(
            self.builder,
            timeStamp=util_fb.createTimeStamp(self.builder, self.time, 0.0),
            frame="map",
            projectUuid=self.projectUuid,
        )

        encoding = self.builder.CreateString(self.IMAGE_ENCODING)

        Image.Start(self.builder)
        Image.AddHeader(self.builder, header)
        Image.AddData(self.builder, imData)
        Image.AddEncoding(self.builder, encoding)
        Image.AddHeight(self.builder, img.shape[0])
        Image.AddWidth(self.builder, img.shape[1])
        Image.AddIsBigendian(self.builder, False)
        if len(labels) > 0:
            Image.AddLabelsBb(self.builder, boundingBox2DLabeledWithCategoryVector)
        Image.AddLabelsGeneral(self.builder, labelsGeneralCat)
        Image.AddStep(self.builder, 3 * img.shape[1])
        return Image.End(self.builder)

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
