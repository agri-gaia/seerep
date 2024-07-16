from typing import List

from flatbuffers import Builder
from grpc import Channel
from seerep.fb import (
    Empty,
    Image,
    PointCloud2,
    PointStamped,
    ProjectInfos,
    Query,
    QueryInstance,
    UuidsPerProject,
)
from seerep.fb import image_service_grpc_fb as image_srv_fb
from seerep.fb import instance_service_grpc_fb as instance_srv_fb
from seerep.fb import meta_operations_grpc_fb as meta_ops_srv_fb
from seerep.fb import point_cloud_service_grpc_fb as pcl2_srv_fb
from seerep.fb import point_service_grpc_fb as point_srv_fb
from seerep.util.common import get_gRPC_channel


class ServiceManager:
    def __init__(self, grpc_channel: Channel = get_gRPC_channel()) -> None:
        self._channel = grpc_channel

    @property
    def channel(self) -> Channel:
        return self._channel

    def get_stub(self, stub_type):
        if not getattr(self, f"_stub_{stub_type.__name__}", False):
            setattr(
                self, f"_stub_{stub_type.__name__}", stub_type(self._channel)
            )
        return getattr(self, f"_stub_{stub_type.__name__}", None)

    def call_get_instances_fb(
        self, builder: Builder, query_instance: QueryInstance.QueryInstance
    ) -> UuidsPerProject.UuidsPerProject:
        builder.Finish(query_instance)
        buf = builder.Output()
        response_buf = self.get_stub(
            instance_srv_fb.InstanceServiceStub
        ).GetInstances(bytes(buf))
        return UuidsPerProject.UuidsPerProject.GetRootAs(response_buf)

    def call_get_project_fb(
        self, builder: Builder
    ) -> ProjectInfos.ProjectInfos:
        Empty.Start(builder)
        empty_msg = Empty.End(builder)
        builder.Finish(empty_msg)
        buf = builder.Output()
        stub = self.get_stub(meta_ops_srv_fb.MetaOperationsStub)
        response_buf = stub.GetProjects(bytes(buf))
        return ProjectInfos.ProjectInfos.GetRootAs(response_buf)

    def call_get_pointcloud2_fb(
        self, builder: Builder, query: Query.Query
    ) -> List[PointCloud2.PointCloud2]:
        builder.Finish(query)
        buf = builder.Output()
        pcl_list = []
        stub = self.get_stub(pcl2_srv_fb.PointCloudServiceStub)

        for pcl in stub.GetPointCloud2(bytes(buf)):
            pcl_list.append(PointCloud2.PointCloud2.GetRootAs(pcl))
        return pcl_list

    def call_get_points_fb(
        self, builder: Builder, query: Query.Query
    ) -> List[PointStamped.PointStamped]:
        builder.Finish(query)
        buf = builder.Output()
        point_lst: List[PointStamped.PointStamped] = []
        stub = self.get_stub(point_srv_fb.PointServiceStub)

        for response in stub.GetPoint(bytes(buf)):
            point_lst.append(PointStamped.PointStamped.GetRootAs(response))
        return point_lst

    def call_get_images_fb(
        self, builder: Builder, query: Query.Query
    ) -> List[Image.Image]:
        builder.Finish(query)
        buf = builder.Output()
        queried_images = []
        stub = self.get_stub(image_srv_fb.ImageServiceStub)

        for response in stub.GetImage(bytes(buf)):
            queried_images.append(Image.Image.GetRootAs(response))
        return queried_images
