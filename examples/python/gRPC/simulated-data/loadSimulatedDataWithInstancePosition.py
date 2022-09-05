#!/usr/bin/env python3

import os
import sys

import flatbuffers
import imageio.v2 as imageio
import numpy as np
import quaternion
import yaml
from fb import (
    Boundingbox2D,
    BoundingBox2DLabeled,
    Header,
    Image,
    LabelWithInstance,
    Point,
    Point2D,
    PointStamped,
    Quaternion,
    Timestamp,
    Transform,
    TransformStamped,
    Vector3,
)
from fb import image_service_grpc_fb as imageService
from fb import point_service_grpc_fb as pointService
from fb import tf_service_grpc_fb as tfService

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import util
import util_fb


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

        self.channel = util.get_gRPC_channel()
        self.projectUuid = util_fb.get_or_create_project(self.channel, self.PROJECT_NAME, True)
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

    def __createLabelWithInstance(self, builder, label, theInstanceUuid):
        instanceUuid = builder.CreateString(theInstanceUuid)
        labelString = builder.CreateString(label)
        LabelWithInstance.Start(builder)
        LabelWithInstance.AddInstanceUuid(builder, instanceUuid)
        LabelWithInstance.AddLabel(builder, labelString)
        return LabelWithInstance.End(builder)

    def __createPoint2D(self, builder, x, y):
        Point2D.Start(builder)
        Point2D.AddX(builder, x)
        Point2D.AddY(builder, y)
        return Point2D.End(builder)

    def __createPoint3D(self, builder, x, y, z):
        Point.Start(builder)
        Point.AddX(builder, x)
        Point.AddY(builder, y)
        Point.AddZ(builder, z)
        return Point.End(builder)

    def __createBoundingBox2D(self, builder, x, y, xExtent, yExtent):
        pointMin = self.__createPoint2D(builder, x - xExtent / 2.0, y - yExtent / 2.0)
        pointMax = self.__createPoint2D(builder, x + xExtent / 2.0, y + yExtent / 2.0)

        Boundingbox2D.Start(builder)
        Boundingbox2D.AddPointMin(builder, pointMin)
        Boundingbox2D.AddPointMax(builder, pointMax)
        return Boundingbox2D.End(builder)

    def __createBoundingBox2DLabeled(self, builder, boundingBox2D, labelWithInstance):
        BoundingBox2DLabeled.Start(builder)
        BoundingBox2DLabeled.AddBoundingBox(builder, boundingBox2D)
        BoundingBox2DLabeled.AddLabelWithInstance(builder, labelWithInstance)
        return BoundingBox2DLabeled.End(builder)

    def __listToImageLabelsVector(self, builder, dataList):
        Image.StartLabelsBbVector(builder, len(dataList))
        for data in dataList:
            builder.PrependUOffsetTRelative(data)
        return builder.EndVector()

    def __listToPointStampedLabelsVector(self, builder, dataList):
        PointStamped.StartLabelsGeneralVector(builder, len(dataList))
        for data in dataList:
            builder.PrependUOffsetTRelative(data)
        return builder.EndVector()

    def __createPointForInstance(self, pointCloudData, instanceUuid, label, imageX, imageY, time, frame, img):
        builder = flatbuffers.Builder(1024)
        pixelX = int(img.shape[0] * imageX)
        pixelY = int(img.shape[1] * imageY)
        position = pointCloudData[pixelX][pixelY]
        point = self.__createPoint3D(builder, position[0], position[1], position[2])

        labelGeneral = self.__createLabelWithInstance(builder, label, instanceUuid)
        labelGeneralVector = self.__listToPointStampedLabelsVector(builder, [labelGeneral])

        header = self.__createHeader(builder, time, frame)

        PointStamped.Start(builder)
        PointStamped.AddPoint(builder, point)
        PointStamped.AddHeader(builder, header)
        PointStamped.AddLabelsGeneral(builder, labelGeneralVector)
        pointStamped = PointStamped.End(builder)
        builder.Finish(pointStamped)

        self.points.append(bytes(builder.Output()))

    def __createBoundingBox2dLabeledVectorMsg(self, builder, baseAnnotationPath, basePointCloudPath, time, frame, img):
        annotations = np.genfromtxt(baseAnnotationPath + self.ANNOTATIONS_FILE_EXTENSION, dtype="<U40")
        pointCloudData = np.load(basePointCloudPath + self.POINTCLOUD_FILE_EXTENSION)
        boundingBox2dLabeledVector = []
        for a in annotations:
            labelFloat, x, y, xExtent, yExtent = a[:-1].astype("float64")
            instanceUuid = a[-1]
            print("uuid: " + instanceUuid)

            label = self.labelSwitch.get(int(round(labelFloat)))
            labelWithInstance = self.__createLabelWithInstance(builder, label, instanceUuid)
            boundingBox2D = self.__createBoundingBox2D(builder, x, y, xExtent, yExtent)
            boundingBox2dLabeledVector.append(
                self.__createBoundingBox2DLabeled(builder, boundingBox2D, labelWithInstance)
            )

            if instanceUuid not in self.instanceUuidSet:
                self.__createPointForInstance(pointCloudData, instanceUuid, label, x, y, time, frame, img)
                self.instanceUuidSet.append(instanceUuid)

        return self.__listToImageLabelsVector(builder, boundingBox2dLabeledVector)

    def __createTimeStamp(self, builder, time):
        Timestamp.Start(builder)
        Timestamp.AddSeconds(builder, time)
        Timestamp.AddNanos(builder, 0)
        return Timestamp.End(builder)

    def __createHeader(self, builder, time, frame):
        timeStamp = self.__createTimeStamp(builder, time)

        frameId = builder.CreateString(frame)
        projectUuidStr = builder.CreateString(self.projectUuid)
        Header.Start(builder)
        Header.AddFrameId(builder, frameId)
        Header.AddStamp(builder, timeStamp)
        Header.AddUuidProject(builder, projectUuidStr)
        return Header.End(builder)

    def __createImage(self, builder, image, header, boundingBox2dLabeledVectorMsg):
        encoding = builder.CreateString(self.IMAGE_ENCODING)

        imData = builder.CreateByteVector(image.tobytes())
        Image.Start(builder)
        Image.AddHeader(builder, header)
        Image.AddHeight(builder, image.shape[0])
        Image.AddWidth(builder, image.shape[1])
        Image.AddEncoding(builder, encoding)
        Image.AddStep(builder, 3 * image.shape[1])
        Image.AddData(builder, imData)
        Image.AddLabelsBb(builder, boundingBox2dLabeledVectorMsg)
        return Image.End(builder)

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

                builder = flatbuffers.Builder(1024)

                timeThisIteration = self.__getCurrentTime(folderIndex, i)

                baseFilePath = self.__createBasePath(imagePath, i)
                baseAnnotationPath = self.__createBasePath(annotationPath, i)
                basePointcloudPath = self.__createBasePath(pointcloudPath, i)

                img = imageio.imread(baseFilePath + self.IMAGE_FILE_EXTENSION)

                boundingBox2dLabeledVectorMsg = self.__createBoundingBox2dLabeledVectorMsg(
                    builder, baseAnnotationPath, basePointcloudPath, timeThisIteration, self.CAMERA_FRAME, img
                )

                header = self.__createHeader(builder, timeThisIteration, self.CAMERA_FRAME)

                imageMsg = self.__createImage(builder, img, header, boundingBox2dLabeledVectorMsg)

                builder.Finish(imageMsg)
                yield bytes(builder.Output())

    def __createVector3(self, builder, t):
        Vector3.Start(builder)
        Vector3.AddX(builder, t[0])
        Vector3.AddY(builder, t[1])
        Vector3.AddZ(builder, t[2])
        return Vector3.End(builder)

    def __createQuaternion(self, builder, quat):
        Quaternion.Start(builder)
        Quaternion.AddX(builder, quat.x)
        Quaternion.AddY(builder, quat.y)
        Quaternion.AddZ(builder, quat.z)
        Quaternion.AddW(builder, quat.w)
        return Quaternion.End(builder)

    def __createTransformFromQuaternion(self, builder, extrinsics):

        calib_e = yaml.safe_load(extrinsics)["camera_pose"]
        calib_e = np.linalg.inv(np.matrix(calib_e))

        r = calib_e[:3, :3]
        quat = quaternion.from_rotation_matrix(r)
        rotation = self.__createQuaternion(builder, quat)

        t = calib_e[:3, -1]
        translation = self.__createVector3(builder, t)

        Transform.Start(builder)
        Transform.AddTranslation(builder, translation)
        Transform.AddRotation(builder, rotation)
        return Transform.End(builder)

    def __createTransformStamped(self, builder, childFrame, headerTf, transform):
        childFrame = builder.CreateString(childFrame)
        TransformStamped.Start(builder)
        TransformStamped.AddChildFrameId(builder, childFrame)
        TransformStamped.AddHeader(builder, headerTf)
        TransformStamped.AddTransform(builder, transform)
        return TransformStamped.End(builder)

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
                        headerTf = self.__createHeader(builder, timeThisIteration, self.MAP_FRAME)
                        tf = self.__createTransformStamped(builder, self.CAMERA_FRAME, headerTf, transform)

                        builder.Finish(tf)
                        yield bytes(builder.Output())

                    except yaml.YAMLError as exc:
                        print(exc)


if __name__ == "__main__":
    LoadSimulatedDataWithInstancePosition()
