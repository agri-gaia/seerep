from seerep.util.common import get_gRPC_channel
from seerep.util.helper.msgs import FbQuery, FbQueryInstance
from seerep.util.helper.service_manager import ServiceManager

if __name__ == "__main__":
    grpc_channel = get_gRPC_channel()

    query_builder = FbQuery(grpc_channel)

    query_instance_builder = FbQueryInstance(grpc_channel)

    uuids = ServiceManager().call_get_instances_fb(
        query_instance_builder.builder, query_instance_builder.datatype_instance
    )

    if uuids.UuidsPerProjectLength() > 0:
        for idx in range(uuids.UuidsPerProject(0).UuidsLength()):
            print(uuids.UuidsPerProject(0).Uuids(idx))
        print(uuids.UuidsPerProject(0).UuidsLength())
