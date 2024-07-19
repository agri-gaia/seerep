import time
from typing import List
from uuid import uuid4

import flatbuffers
from grpc import Channel
from seerep.fb import (
    camera_intrinsics_service_grpc_fb as camera_intrinsic_service,
)
from seerep.fb import image_service_grpc_fb as image_service
from seerep.fb.Image import Image
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createCameraIntrinsics,
    createHeader,
    createImage,
    createProject,
    createQuery,
    createRegionOfInterest,
    createTimeStamp,
)


def add_image(
    grpc_channel: Channel, project_uuid: str, camera_uuid: str
) -> None:
    fbb = flatbuffers.Builder()
    image_service_stub = image_service.ImageServiceStub(grpc_channel)
    timestamp = createTimeStamp(
        fbb, *list(map(int, str(time.time()).split(".")))
    )
    header = createHeader(fbb, timestamp, "map", project_uuid)
    image = createImage(
        fbb,
        header,
        "rgb8",
        True,
        camera_uuid,
        height=480,
        width=640,
        uri="file:///path/to/image.jpg",
    )
    fbb.Finish(image)
    image_service_stub.TransferImage(iter([bytes(fbb.Output())]))


def get_image(grpc_channel: Channel, project_uuid: str) -> List[bytearray]:
    fbb = flatbuffers.Builder()
    image_service_stub = image_service.ImageServiceStub(grpc_channel)
    query = createQuery(
        fbb,
        projectUuids=[fbb.CreateString(project_uuid)],
    )
    fbb.Finish(query)
    return image_service_stub.GetImage(bytes(fbb.Output()))


def create_project(grpc_channel: Channel, project_name: str) -> str:
    fbb = flatbuffers.Builder()
    return createProject(
        grpc_channel, fbb, project_name, "map", "WGS84", 0.0, 85.0, 23.0
    )


def create_cameraintrinsics(grpc_channel: Channel, project_uuid: str) -> str:
    fbb = flatbuffers.Builder()
    camera_intrinsic_service_stub = (
        camera_intrinsic_service.CameraIntrinsicsServiceStub(grpc_channel)
    )
    timestamp = createTimeStamp(fbb, 0, 0)
    ci_uuid = str(uuid4())
    header = createHeader(fbb, timestamp, "map", project_uuid, ci_uuid)
    roi = createRegionOfInterest(fbb, 0, 0, 0, 0, False)

    distortion_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    rect_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    intrins_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    proj_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

    cameraintrinsics = createCameraIntrinsics(
        fbb,
        header,
        3,
        4,
        "plumb_bob",
        distortion_matrix,
        intrins_matrix,
        rect_matrix,
        proj_matrix,
        4,
        5,
        roi,
        5,
    )
    fbb.Finish(cameraintrinsics)
    camera_intrinsic_service_stub.TransferCameraIntrinsics(bytes(fbb.Output()))
    return ci_uuid


if __name__ == "__main__":
    grpc_channel = get_gRPC_channel()
    project_uuid = create_project(grpc_channel, "uri_storage_project")
    camera_intrinsic_uuid = create_cameraintrinsics(grpc_channel, project_uuid)

    add_image(
        grpc_channel,
        project_uuid,
        camera_intrinsic_uuid,
    )

    images = get_image(grpc_channel, project_uuid)
    if not images:
        print("No images found")
    else:
        for img_buf in images:
            img = Image.GetRootAs(img_buf)
            print(
                f"Received image with uuid: {img.Header().UuidMsgs().decode()}"
            )
            print(f"Path to image is '{img.Uri().decode()}'")
