from seerep.util.common import get_gRPC_channel
from seerep.util.helper.msgs import EnumFbQueryInstance, FbQueryInstance
from seerep.util.helper.service_manager import ServiceManager

if __name__ == "__main__":
    query_instance_builder = FbQueryInstance(
        get_gRPC_channel(), enum_types=set([EnumFbQueryInstance.QUERY])
    )
    query_instance = query_instance_builder.datatype_instance
    uuids = ServiceManager().call_get_instances_fb(
        query_instance_builder.builder, query_instance
    )
    for idx in range(uuids.UuidsPerProject(0).UuidsLength()):
        print(uuids.UuidsPerProject(0).Uuids(idx))
    print(uuids.UuidsPerProject(0).UuidsLength())
