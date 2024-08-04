import time
import uuid
from typing import List, Optional, Tuple, Union

import flatbuffers
import gRPC.images.gRPC_fb_addCameraIntrinsics as add_ci
import numpy as np
from google.protobuf import empty_pb2
from grpc import Channel
from quaternion import quaternion
from seerep.fb import (
    CameraIntrinsics,
    Image,
)
from seerep.fb import image_service_grpc_fb as image_service
from seerep.fb import tf_service_grpc_fb as tf_service
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createHeader,
    createQuaternion,
    createTimeStamp,
    createTransform,
    createTransformStamped,
    createVector3,
)

# only counts when the timestamps parameter is not None
N_IMAGES = 10


def send_labeled_images_raw(
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
    timestamps: Union[List[Tuple[int, int]], None] = None,
    frame_id: str = "map",
    child_frame_id: str = "camera",
) -> List[bytes]:
    builder = flatbuffers.Builder(1000)

    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)

    # 3. Check if we have an existing test project, if not, one is created.
    if target_proj_uuid is None:
        # 2. Get all projects from the server
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            print(project.name + " " + project.uuid)
            if project.name == "testproject":
                target_proj_uuid = project.uuid

        if target_proj_uuid is None:
            response = stubMeta.CreateProject(
                projectCreation_pb2.ProjectCreation(
                    name="testproject", mapFrameId="map"
                )
            )
            target_proj_uuid = response.uuid

    # 2. Check if the defined project exist; if not return None
    if target_proj_uuid is None:
        print("could not create project and add camera intrinsics to it!")
        return None

    camins: CameraIntrinsics.CameraIntrinsics = add_ci.add_camintrins(
        None, target_proj_uuid, grpc_channel, frame_id
    )
    cam_uuid = camins.Header().UuidMsgs().decode()

    images = []

    it_len = timestamps if timestamps is not None else N_IMAGES

    for n in range(len(it_len)):
        lim = 16  # 16 x 16 pixels
        rgb = np.ndarray((lim, lim, 3), np.ubyte)
        for i in range(lim):
            for j in range(lim):
                x = float(i) / lim
                y = float(j) / lim
                z = float(j) / lim
                r = np.ubyte((x * 255.0 + n) % 255)
                g = np.ubyte((y * 255.0 + n) % 255)
                b = np.ubyte((z * 255.0 + n) % 255)
                rgb[i, j, 0] = r
                rgb[i, j, 1] = g
                rgb[i, j, 2] = b

        if timestamps is None:
            timestamp = createTimeStamp(builder, int(time.time()))
        else:
            timestamp = createTimeStamp(
                builder, timestamps[n][0], timestamps[n][1]
            )

        header = createHeader(
            builder,
            timestamp,
            child_frame_id,
            target_proj_uuid,
            str(uuid.uuid4()),
        )
        enc = builder.CreateString("rgb8")
        im_data = builder.CreateByteVector(rgb.tobytes())
        ciuuid = builder.CreateString(cam_uuid)

        Image.Start(builder)
        Image.AddHeader(builder, header)
        Image.AddData(builder, im_data)
        Image.AddEncoding(builder, enc)
        Image.AddHeight(builder, rgb.shape[0])
        Image.AddWidth(builder, rgb.shape[1])
        Image.AddIsBigendian(builder, False)
        Image.AddStep(builder, 3 * rgb.shape[1])
        Image.AddUuidCameraintrinsics(builder, ciuuid)
        im = Image.End(builder)
        builder.Finish(im)

        images.append(bytes(builder.Output()))

    image_service.ImageServiceStub(grpc_channel).TransferImage(iter(images))
    transfer_tfs(
        grpc_channel, target_proj_uuid, timestamps, frame_id, child_frame_id
    )
    return images


def stream_tfs(
    timestamps: List[Tuple[int, int]],
    target_project_uuid: str,
    frame_id: str,
    child_frame_id: str,
):
    builder = flatbuffers.Builder(1024)
    quat = createQuaternion(builder, quaternion(1, 0, 0, 0))
    for idx, (seconds, nanos) in enumerate(timestamps):
        timestmp = createTimeStamp(builder, seconds, nanos)

        # when not giving a uuid the corresponding retrieved tf from server has
        # no uuid msg
        header = createHeader(
            builder, timestmp, frame_id, target_project_uuid, str(uuid.uuid4())
        )

        translation = createVector3(
            builder,
            np.array([100 * idx**2, 100 * idx**2, 30 * idx], dtype=np.float64),
        )

        tf = createTransform(builder, translation, quat)
        tf_s = createTransformStamped(builder, child_frame_id, header, tf)

        builder.Finish(tf_s)
        yield bytes(builder.Output())


def transfer_tfs(
    grpc_channel: Channel,
    target_project_uuid: str,
    timestamps: List[Tuple[int, int]],
    frame_id: str = "map",
    child_frame_id: str = "camera",
):
    stub = tf_service.TfServiceStub(grpc_channel)

    stub.TransferTransformStamped(
        stream_tfs(timestamps, target_project_uuid, frame_id, child_frame_id)
    )


def send_labeled_images(
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
    timestamps: Union[List[Tuple[int, int]], None] = None,
    frame_id: str = "map",
    child_frame_id: str = "camera",
) -> List[Image.Image]:
    return [
        Image.Image.GetRootAs(img)
        for img in send_labeled_images_raw(
            target_proj_uuid, grpc_channel, timestamps, frame_id, child_frame_id
        )
    ]


if __name__ == "__main__":
    timestamp_nanos = 1245
    nanos_factor = 1e-9

    timestamps = [
        (t, timestamp_nanos) for t in range(1661336507, 1661336558, 10)
    ]
    send_labeled_images(timestamps=timestamps)
    print("sent images")
