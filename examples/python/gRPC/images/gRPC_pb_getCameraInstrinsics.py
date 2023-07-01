from seerep.pb import camera_intrinsics_query_pb2 as cameraintrinsicsquery
from seerep.pb import camera_intrinsics_service_pb2_grpc as camintrinsics_service
from seerep.util.common import get_gRPC_channel


def query_camintrinsics(projectuuid, ciuuid, grpc_channel=get_gRPC_channel()):

    stub = camintrinsics_service.CameraIntrinsicsServiceStub(grpc_channel)

    # Fetch the CI
    ci_query = cameraintrinsicsquery.CameraIntrinsicsQuery()

    ci_query.uuid_camera_intrinsics = ciuuid
    ci_query.uuid_project = projectuuid

    fetched_camintrinsics = stub.GetCameraIntrinsics(ci_query)
    print(fetched_camintrinsics)
    return fetched_camintrinsics
