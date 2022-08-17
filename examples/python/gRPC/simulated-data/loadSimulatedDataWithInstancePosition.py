#!/usr/bin/env python3

import os
import sys
import uuid

import flatbuffers
import imageio
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

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util
import util_fb


class LoadSimulatedDataWithInstancePosition:
    def __init__(self) -> None:
        self.numImages = 80
        projectName = "simulatedDataWithInstances"

        self.channel = util.get_gRPC_channel()
        self.projectUuid = util_fb.get_or_create_project(self.channel, projectName, True)
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

        self.stubTf = tfService.TfServiceStub(self.channel)
        self.stubImage = imageService.ImageServiceStub(self.channel)
        self.stubPoint = pointService.PointServiceStub(self.channel)

        self.stubImage.TransferImage(self.__loadImages())
        self.stubTf.TransferTransformStamped(self.__loadTf())

    def __createBoundingBox2dLabeledVectorMsg(self, builder, baseAnnotationPath, basePointcloudPath):
        annotations = np.genfromtxt(baseAnnotationPath + ".txt", delimiter=" ")
        pointcloudData = np.load(basePointcloudPath + ".npy")
        boundingBox2dLabeledVector = []
        for a in annotations:
            theInstanceUuid = str(uuid.uuid4())
            theLabel = self.labelSwitch.get(int(round(a[0])))
            instanceUuid = builder.CreateString(theInstanceUuid)
            labelString = builder.CreateString(theLabel)
            LabelWithInstance.Start(builder)
            LabelWithInstance.AddInstanceUuid(builder, instanceUuid)
            LabelWithInstance.AddLabel(builder, labelString)
            labelWithInstance = LabelWithInstance.End(builder)

            Point2D.Start(builder)
            Point2D.AddX(builder, a[1] - a[3] / 2.0)
            Point2D.AddX(builder, a[2] - a[4] / 2.0)
            pointMin = Point2D.End(builder)
            Point2D.Start(builder)
            Point2D.AddX(builder, a[1] + a[3] / 2.0)
            Point2D.AddX(builder, a[2] + a[4] / 2.0)
            pointMax = Point2D.End(builder)

            Boundingbox2D.Start(builder)
            Boundingbox2D.AddPointMin(builder, pointMin)
            Boundingbox2D.AddPointMax(builder, pointMax)
            boundingBox2D = Boundingbox2D.End(builder)

            BoundingBox2DLabeled.Start(builder)
            BoundingBox2DLabeled.AddBoundingBox(builder, boundingBox2D)
            BoundingBox2DLabeled.AddLabelWithInstance(builder, labelWithInstance)
            boundingBox2dLabeled = BoundingBox2DLabeled.End(builder)
            boundingBox2dLabeledVector.append(boundingBox2dLabeled)

        Image.StartLabelsBbVector(builder, len(boundingBox2dLabeledVector))
        for bb2 in boundingBox2dLabeledVector:
            builder.PrependUOffsetTRelative(bb2)
        return builder.EndVector()

    def __loadImages(self):
        # get and save the time

        for folderIndex in range(len(self.root)):
            imagePath = self.root[folderIndex] + "camera_main_camera/rect/"
            annotationPath = self.root[folderIndex] + "camera_main_camera_annotations/bounding_box/"
            pointcloudPath = self.root[folderIndex] + "camera_main_camera_annotations/pcl/"

            for i in range(self.numImages):
                builder = flatbuffers.Builder(1024)

                timeThisIteration = self.theTime[folderIndex] + i

                baseFilePath = imagePath + str(i).zfill(4)
                baseAnnotationPath = annotationPath + str(i).zfill(4)
                basePointcloudPath = pointcloudPath + str(i).zfill(4)

                boundingBox2dLabeledVectorMsg = self.__createBoundingBox2dLabeledVectorMsg(
                    builder, baseAnnotationPath, basePointcloudPath
                )

                Timestamp.Start(builder)
                Timestamp.AddSeconds(builder, timeThisIteration)
                Timestamp.AddNanos(builder, 0)
                timeStamp = Timestamp.End(builder)

                frameId = builder.CreateString("camera")
                projectUuidStr = builder.CreateString(self.projectUuid)
                Header.Start(builder)
                Header.AddFrameId(builder, frameId)
                Header.AddStamp(builder, timeStamp)
                Header.AddUuidProject(builder, projectUuidStr)
                header = Header.End(builder)

                encoding = builder.CreateString("rgb8")
                im = imageio.imread(baseFilePath + ".png")
                imData = builder.CreateByteVector(im.tobytes())
                Image.Start(builder)
                Image.AddHeader(builder, header)
                Image.AddHeight(builder, im.shape[0])
                Image.AddWidth(builder, im.shape[1])
                Image.AddEncoding(builder, encoding)
                Image.AddStep(builder, 3 * im.shape[1])
                Image.AddData(builder, imData)
                Image.AddLabelsBb(builder, boundingBox2dLabeledVectorMsg)
                imageMsg = Image.End(builder)

                builder.Finish(imageMsg)
                yield bytes(builder.Output())

    def __loadTf(self):
        for folderIndex in range(len(self.root)):
            imagePath = self.root[folderIndex] + "camera_main_camera/extrinsics/"

            for i in range(self.numImages):  # 80
                builder = flatbuffers.Builder(1024)
                timeThisIteration = self.theTime[folderIndex] + i
                baseFilePath = imagePath + str(i).zfill(4)
                with open(baseFilePath + ".yaml", "r") as stream:
                    try:
                        calib_e = yaml.safe_load(stream)["camera_pose"]
                        calib_e = np.linalg.inv(np.matrix(calib_e))
                        r = calib_e[:3, :3]
                        t = calib_e[:3, -1]

                        quat = quaternion.from_rotation_matrix(r)

                        Timestamp.Start(builder)
                        Timestamp.AddSeconds(builder, timeThisIteration)
                        Timestamp.AddNanos(builder, 0)
                        timeStamp = Timestamp.End(builder)

                        frameId = builder.CreateString("map")
                        projectUuidHeader = builder.CreateString(self.projectUuid)
                        Header.Start(builder)
                        Header.AddFrameId(builder, frameId)
                        Header.AddStamp(builder, timeStamp)
                        Header.AddUuidProject(builder, projectUuidHeader)
                        headerTf = Header.End(builder)

                        Vector3.Start(builder)
                        Vector3.AddX(builder, t[0])
                        Vector3.AddY(builder, t[1])
                        Vector3.AddZ(builder, t[2])
                        translation = Vector3.End(builder)

                        Quaternion.Start(builder)
                        Quaternion.AddX(builder, quat.x)
                        Quaternion.AddY(builder, quat.y)
                        Quaternion.AddZ(builder, quat.z)
                        Quaternion.AddW(builder, quat.w)
                        rotation = Quaternion.End(builder)

                        Transform.Start(builder)
                        Transform.AddTranslation(builder, translation)
                        Transform.AddRotation(builder, rotation)
                        transform = Transform.End(builder)

                        childFrame = builder.CreateString("camera")
                        TransformStamped.Start(builder)
                        TransformStamped.AddChildFrameId(builder, childFrame)
                        TransformStamped.AddHeader(builder, headerTf)
                        TransformStamped.AddTransform(builder, transform)
                        tf = TransformStamped.End(builder)

                        builder.Finish(tf)
                        yield bytes(builder.Output())

                    except yaml.YAMLError as exc:
                        print(exc)


if __name__ == "__main__":
    LoadSimulatedDataWithInstancePosition()
