#!/usr/bin/env python3

import flatbuffers
import imageio.v2 as imageio
import numpy as np
import quaternion
import yaml
from seerep.fb import Transform
from seerep.fb import image_service_grpc_fb as imageService
from seerep.fb import point_service_grpc_fb as pointService
from seerep.fb import tf_service_grpc_fb as tfService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import getOrCreateProject


class LoadSimulatedDataWithInstancePosition:
    def __init__(self) -> None:
        self.IMAGE_SUBPATH = "camera_main_camera/rect/"
        self.BOUNDINGBOX_SUBPATH = "camera_main_camera_annotations/bounding_box_uuid/"
        self.POINTCLOUD_SUBPATH = "camera_main_camera_annotations/pcl/"
        self.EXTRINSICS_SUBPATH = "camera_main_camera/extrinsics/"

        self.MAP_FRAME = "map"
        self.CAMERA_FRAME = "camera"

        self.ANNOTATIONS_FILE_EXTENSION = ".txt"
        self.POINTCLOUD_FILE_EXTENSION = ".npy"
        self.IMAGE_FILE_EXTENSION = ".png"
        self.EXTRINSICS_FILE_EXTENSION = ".yaml"

        self.IMAGE_ENCODING = "rgb8"

        self.NUM_IMAGES = 80
        self.PROJECT_NAME = "simulatedDataWithInstances"

        self.labelCategory = "ground_truth"

        self.builder = flatbuffers.Builder(1024)
        self.channel = get_gRPC_channel()
        self.projectUuid = getOrCreateProject(self.builder, self.channel, self.PROJECT_NAME)
        self.theTime = [1654688921]  # 08.06.2022 13:49 #int(time.time())
        self.root = ["/seerep/seerep-data/cw_synthetic_maize/dataset_v2/"]

        self.labelSwitch = {
            1: "Mais",
            2: "Herbst-Löwenzahn",
            3: "Gänseblümchen",
            4: "Krauser Ampfer",
            5: "Giersch",
            6: "Spitzwegerich",
        }

        self.points = []
        self.instanceUuidSet = []

        self.stubTf = tfService.TfServiceStub(self.channel)
        self.stubImage = imageService.ImageServiceStub(self.channel)
        self.stubPoint = pointService.PointServiceStub(self.channel)

        self.stubImage.TransferImage(self.__loadImages())
        self.stubTf.TransferTransformStamped(self.__loadTf())
        self.stubPoint.TransferPoint(iter(self.points))

    def __createPointForInstance(self, pointCloudData, instanceUuid, label, imageX, imageY, time, frame, img):
        builder = flatbuffers.Builder(1024)
        pixelX = int(img.shape[1] * imageX)
        pixelY = int(img.shape[0] * imageY)
        position = pointCloudData[pixelY][pixelX]
        point = util_fb.createPoint(builder, position[0], position[1], position[2])

        labelGeneral = []
        labelGeneral.append(util_fb.createLabelWithInstance(builder, label, 1.0, instanceUuid))
        labelGeneralCategory = util_fb.createLabelWithCategory(builder, [self.labelCategory], [labelGeneral])

        timestamp = util_fb.createTimeStamp(builder, time)
        header = util_fb.createHeader(builder, timestamp, frame, self.projectUuid)

        pointStamped = util_fb.createPointStamped(builder, point, header, labelGeneralCategory)
        builder.Finish(pointStamped)

        self.points.append(bytes(builder.Output()))

    def __createBoundingBox2dCategory(self, builder, baseAnnotationPath, basePointCloudPath, time, frame, img):
        annotations = np.genfromtxt(baseAnnotationPath + self.ANNOTATIONS_FILE_EXTENSION, dtype="<U40")
        pointCloudData = np.load(basePointCloudPath + self.POINTCLOUD_FILE_EXTENSION)
        boundingBox2dLabeledVector = []
        for a in annotations:
            labelFloat, x, y, xExtent, yExtent = a[:-1].astype("float64")
            instanceUuid = a[-1]
            print("uuid: " + instanceUuid)

            label = self.labelSwitch.get(int(round(labelFloat)))
            labelWithInstance = util_fb.createLabelWithInstance(builder, label, 1.0, instanceUuid)
            centerPoint = util_fb.createPoint2d(builder, x, y)
            spatialExtent = util_fb.createPoint2d(builder, xExtent, yExtent)
            boundingBox2D = util_fb.createBoundingBox2d(builder, centerPoint, spatialExtent)

            boundingBox2dLabeledVector.append(
                util_fb.createBoundingBox2dLabeled(builder, labelWithInstance, boundingBox2D)
            )

            if instanceUuid not in self.instanceUuidSet:
                self.__createPointForInstance(pointCloudData, instanceUuid, label, x, y, time, frame, img)
                self.instanceUuidSet.append(instanceUuid)
        categoryString = builder.CreateString(self.labelCategory)
        boundingBox2dCategoryVector = []
        boundingBox2dCategoryVector.append(
            util_fb.createBoundingBox2DLabeledWithCategory(self.builder, categoryString, boundingBox2dLabeledVector)
        )
        return boundingBox2dCategoryVector

    def __createBasePath(self, path, i):
        return path + str(i).zfill(4)

    def __getCurrentTime(self, folderIndex, i):
        return self.theTime[folderIndex] + i

    def __getPath(self, folderIndex, subPath):
        return self.root[folderIndex] + subPath

    def __loadImages(self):
        # get and save the time

        for folderIndex in range(len(self.root)):
            imagePath = self.__getPath(folderIndex, self.IMAGE_SUBPATH)
            annotationPath = self.__getPath(folderIndex, self.BOUNDINGBOX_SUBPATH)
            pointcloudPath = self.__getPath(folderIndex, self.POINTCLOUD_SUBPATH)

            for i in range(self.NUM_IMAGES):
                print("image " + str(i))

                timeThisIteration = self.__getCurrentTime(folderIndex, i)

                baseFilePath = self.__createBasePath(imagePath, i)
                baseAnnotationPath = self.__createBasePath(annotationPath, i)
                basePointcloudPath = self.__createBasePath(pointcloudPath, i)

                img = imageio.imread(baseFilePath + self.IMAGE_FILE_EXTENSION)

                bb2dWithCategory = self.__createBoundingBox2dCategory(
                    self.builder, baseAnnotationPath, basePointcloudPath, timeThisIteration, self.CAMERA_FRAME, img
                )
                timestamp = util_fb.createTimeStamp(self.builder, timeThisIteration)
                header = util_fb.createHeader(self.builder, timestamp, self.CAMERA_FRAME, self.projectUuid)

                imageMsg = util_fb.createImage(self.builder, img, header, self.IMAGE_ENCODING, bb2dWithCategory)

                self.builder.Finish(imageMsg)
                yield bytes(self.builder.Output())

    def __createTransformFromQuaternion(self, builder, extrinsics):

        calib_e = yaml.safe_load(extrinsics)["camera_pose"]
        calib_e = np.linalg.inv(np.matrix(calib_e))

        r = calib_e[:3, :3]
        quat = quaternion.from_rotation_matrix(r)
        rotation = util_fb.createQuaternion(builder, quat)

        t = calib_e[:3, -1]
        translation = util_fb.createVector3(builder, t)

        Transform.Start(builder)
        Transform.AddTranslation(builder, translation)
        Transform.AddRotation(builder, rotation)
        return Transform.End(builder)

    def __loadTf(self):
        for folderIndex in range(len(self.root)):
            extrinsicsPath = self.__getPath(folderIndex, self.EXTRINSICS_SUBPATH)

            for i in range(self.NUM_IMAGES):
                builder = flatbuffers.Builder(1024)
                timeThisIteration = self.__getCurrentTime(folderIndex, i)
                baseFilePath = self.__createBasePath(extrinsicsPath, i)

                with open(baseFilePath + self.EXTRINSICS_FILE_EXTENSION, "r") as extrinsics:
                    try:
                        transform = self.__createTransformFromQuaternion(builder, extrinsics)
                        timestamp = util_fb.createTimeStamp(builder, timeThisIteration)
                        headerTf = util_fb.createHeader(builder, timestamp, self.MAP_FRAME, self.projectUuid)
                        tf = util_fb.createTransformStamped(builder, self.CAMERA_FRAME, headerTf, transform)

                        builder.Finish(tf)
                        yield bytes(builder.Output())

                    except yaml.YAMLError as exc:
                        print(exc)


if __name__ == "__main__":
    LoadSimulatedDataWithInstancePosition()
