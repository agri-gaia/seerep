import random
import time
from typing import List, Union
from uuid import uuid4

import flatbuffers
import numpy as np
from grpc import Channel
from seerep.fb import (
    camera_intrinsics_service_grpc_fb as camera_intrinsic_service,
)
from seerep.fb import image_service_grpc_fb as image_service
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createCameraIntrinsics,
    createHeader,
    createImage,
    createProject,
    createRegionOfInterest,
    createTimeStamp,
)


def send_images(
    grpc_channel: Channel,
    project_uuid: str,
    camera_intrinsic_uuid: str,
    image_payloads: List[Union[np.ndarray, str]],
) -> List[bytes]:
    """
    Send images to a SEEREP project

    Args:
        grpc_channel (Channel): The gRPC channel.
        project_uuid (str): The UUID of the project.
        camera_intrinsic_uuid (str): The UUID of the corresponding camera
        intrinsics.
        image_payloads (List[Union[np.ndarray, str]]): A list of image payloads.
        Each payload can be either a numpy array (actual data) or a string
        (for a remote stroage URI).

    Returns:
        List[bytes]: A list of bytes representing the serialized
        FlatBuffers messages.

    """
    fbb = flatbuffers.Builder()
    image_service_stub = image_service.ImageServiceStub(grpc_channel)

    # Use current time as timestamp
    timestamp = createTimeStamp(
        fbb, *list(map(int, str(time.time()).split(".")))
    )

    fb_msgs = []
    for image in image_payloads:
        header = createHeader(fbb, timestamp, "map", project_uuid, str(uuid4()))
        fb_image = createImage(
            fbb,
            header,
            "rgb8",
            True,
            camera_intrinsic_uuid,
            height=image.shape[0] if type(image) is np.ndarray else 480,
            width=image.shape[1] if type(image) is np.ndarray else 640,
            image=image if type(image) is np.ndarray else None,
            uri=None if type(image) is np.ndarray else image,
        )
        fbb.Finish(fb_image)
        fb_msgs.append(bytes(fbb.Output()))

    image_service_stub.TransferImage(iter(fb_msgs))
    return fb_msgs


def generate_image_ressources(num_images: int) -> List[Union[np.ndarray, str]]:
    """
    Generate an image ressource which can either be a URI or random image data.

    Args:
        num_images (int): The number of payloads to generate.

    Returns:
        List[Union[np.ndarray, str]]: A list of image ressources.
    """
    ressources = []
    for i in range(num_images):
        if random.choice([True, False]):
            ressources.append(f"https://example.com/image{i}.jpg")
        else:
            ressources.append(np.random.rand(640, 480, 3).astype(np.uint8))
    return ressources


def create_project(grpc_channel: Channel, project_name: str) -> str:
    """
    Creates a new project in SEEREP.

    Args:
        grpc_channel (Channel): The gRPC channel to communicate with the server.
        project_name (str): The name of the project to create.

    Returns:
        str: The UUID of the created project.
    """
    fbb = flatbuffers.Builder()
    return createProject(
        grpc_channel, fbb, project_name, "map", "WGS84", 0.0, 85.0, 23.0
    )


def send_cameraintrinsics(grpc_channel: Channel, project_uuid: str) -> str:
    """
    Create and send a camera intrinsics object to SEEREP.

    Args:
        grpc_channel (Channel): The gRPC channel to communicate with the server.
        project_uuid (str): The UUID of the project.

    Returns:
        str: The UUID of the created camera intrinsics.
    """
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
    channel = get_gRPC_channel()
    project_uuid = create_project(channel, "testproject")
    camera_uuid = send_cameraintrinsics(channel, project_uuid)

    img_bufs = send_images(
        channel, project_uuid, camera_uuid, generate_image_ressources(10)
    )

    if img_bufs is not None and len(img_bufs) > 0:
        print("Images sent successfully")
