# NOTE: This file is referenced in the following mkdocs files:
#   pytests-message-abstractions.md
#   python-helpers.md
# If any line changes on this file occur, those files may have to be updated as
# well
from enum import auto
from typing import Dict, List

from flatbuffers import Builder
from grpc import Channel
from gRPC.util.datastructures import FrozenEnum
from gRPC.util.msg_abs.msgs_base import MsgsFb, MsgsFunctions, expect_component
from seerep.fb import (
    Datatype,
    LabelCategory,
    Polygon2D,
    Query,
    QueryInstance,
    TimeInterval,
)
from seerep.util import fb_helper as fbh
from seerep.util.service_manager import ServiceManager

C_MAX_INTEGER: int = 2147483647


class EnumFbQuery(FrozenEnum):
    POLYGON = auto()  # def: None
    POLYGONSENSORPOSITION = auto()  # def: None
    FULLY_ENCAPSULATED = auto()  # def: False
    IN_MAP_FRAME = auto()  # def: True
    TIMEINTERVAL = auto()  # def: None
    LABEL = auto()  # def: None
    SPARQL_QUERY = auto()  # def: None
    ONTOLOGY_URI = auto()  # def: None
    MUST_HAVE_ALL_LABELS = auto()  # def: False
    PROJECTUUID = auto()  # def: None
    INSTANCEUUID = auto()  # def: None
    DATAUUID = auto()  # def: None
    WITHOUTDATA = auto()  # def: False
    MAX_NUM_DATA = auto()  # def: None
    SORT_BY_TIME = auto()  # def: False


class EnumFbQueryInstance(FrozenEnum):
    DATATYPE = auto()  # def: None
    QUERY = auto()  # def: None


class FbQuery(MsgsFb[Query.Query]):
    def _set_enum_func_mapping(self) -> Dict[EnumFbQuery, MsgsFunctions]:
        return {
            EnumFbQuery.POLYGON: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.polygon2d(self.builder)
            ),
            EnumFbQuery.POLYGONSENSORPOSITION: MsgsFunctions(
                # TODO provide a example polygon, differing from POLYGON
                lambda: None,
                lambda: Dtypes.Fb.polygon2d(self.builder),
            ),
            EnumFbQuery.FULLY_ENCAPSULATED: MsgsFunctions(
                lambda: False, lambda: True
            ),
            EnumFbQuery.IN_MAP_FRAME: MsgsFunctions(
                lambda: True, lambda: False
            ),
            EnumFbQuery.TIMEINTERVAL: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.time_interval(self.builder)
            ),
            EnumFbQuery.LABEL: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.label_category(self.builder)
            ),
            EnumFbQuery.SPARQL_QUERY: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.sparql_query(self.builder)
            ),
            EnumFbQuery.ONTOLOGY_URI: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.ontology_uri(self.builder)
            ),
            EnumFbQuery.MUST_HAVE_ALL_LABELS: MsgsFunctions(
                lambda: False, lambda: True
            ),
            EnumFbQuery.PROJECTUUID: MsgsFunctions(
                lambda: None,
                lambda: Dtypes.Fb.projectuuid(self.builder, self.channel),
            ),
            EnumFbQuery.INSTANCEUUID: MsgsFunctions(
                lambda: None, lambda: self.instanceuuid()
            ),
            EnumFbQuery.DATAUUID: MsgsFunctions(
                lambda: None,
                lambda: Dtypes.Fb.datauuid(self.builder, self.channel),
            ),
            EnumFbQuery.WITHOUTDATA: MsgsFunctions(lambda: False, lambda: True),
            EnumFbQuery.MAX_NUM_DATA: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.max_num_data()
            ),
            EnumFbQuery.SORT_BY_TIME: MsgsFunctions(
                lambda: False, lambda: True
            ),
        }

    @expect_component(EnumFbQuery.PROJECTUUID)
    def instanceuuid(self) -> List[int]:
        return Dtypes.Fb.intanceuuid(
            self.builder,
            self.channel,
            self.get_component(EnumFbQuery.PROJECTUUID),
        )

    @expect_component(EnumFbQuery.PROJECTUUID)
    def datauuid(self) -> List[int]:
        return Dtypes.Fb.datauuid(
            self.builder, self.channel, self.get_component(EnumFbQuery.DATAUUID)
        )

    def _assemble_datatype_instance(self):
        polygon = self.get_component(EnumFbQuery.POLYGON)
        polygon_sensor_position = self.get_component(
            EnumFbQuery.POLYGONSENSORPOSITION
        )
        fully_encapsulated = self.get_component(EnumFbQuery.FULLY_ENCAPSULATED)
        in_map_frame = self.get_component(EnumFbQuery.IN_MAP_FRAME)
        timeinterval = self.get_component(EnumFbQuery.TIMEINTERVAL)
        label = self.get_component(EnumFbQuery.LABEL)
        must_have_all_labels = self.get_component(
            EnumFbQuery.MUST_HAVE_ALL_LABELS
        )
        projectuuid = self.get_component(EnumFbQuery.PROJECTUUID)
        instanceuuid = self.get_component(EnumFbQuery.INSTANCEUUID)
        datauuid = self.get_component(EnumFbQuery.DATAUUID)
        withoutdata = self.get_component(EnumFbQuery.WITHOUTDATA)
        sort_by_time = self.get_component(EnumFbQuery.SORT_BY_TIME)

        return fbh.createQuery(
            self.builder,
            timeInterval=timeinterval,
            labels=label,
            mustHaveAllLabels=must_have_all_labels,
            projectUuids=projectuuid,
            instanceUuids=instanceuuid,
            dataUuids=datauuid,
            withoutData=withoutdata,
            polygon2d=polygon,
            polygon2dSensorPos=polygon_sensor_position,
            fullyEncapsulated=fully_encapsulated,
            inMapFrame=in_map_frame,
            sortByTime=sort_by_time,
        )


class FbQueryInstance(MsgsFb[QueryInstance.QueryInstance]):
    def _set_enum_func_mapping(self) -> Dict[FrozenEnum, MsgsFunctions]:
        return {
            EnumFbQueryInstance.DATATYPE: MsgsFunctions(
                lambda: Datatype.Datatype().All,
                lambda: Datatype.Datatype().PointCloud,
            ),
            EnumFbQueryInstance.QUERY: MsgsFunctions(
                lambda: Dtypes.FbDefaults.query(self.channel),
                lambda: self.query(),
            ),
        }

    def query(self):
        features = {
            EnumFbQuery.WITHOUTDATA,
            EnumFbQuery.IN_MAP_FRAME,
            EnumFbQuery.PROJECTUUID,
        }
        return FbQuery(self.channel, self.builder, features).datatype_instance

    def _assemble_datatype_instance(self):
        datatype = self.get_component(EnumFbQueryInstance.DATATYPE)
        query = self.get_component(EnumFbQueryInstance.QUERY)

        return fbh.createQueryInstance(
            self.builder, datatype=datatype, query=query
        )


class DatatypeImplementations:
    class Fb:
        @classmethod
        def polygon2d(cls, builder: Builder) -> Polygon2D.Polygon2D:
            polygon_vertices = []
            polygon_vertices.append(fbh.createPoint2d(builder, 0, 0))
            polygon_vertices.append(fbh.createPoint2d(builder, 0, 100))
            polygon_vertices.append(fbh.createPoint2d(builder, 100, 100))
            polygon_vertices.append(fbh.createPoint2d(builder, 100, 0))
            return fbh.createPolygon2D(builder, 100, 0, polygon_vertices)

        @classmethod
        def parameterized_polygon2d(
            cls, builder: Builder, quad_extent: float, height: float
        ) -> Polygon2D.Polygon2D:
            """
            Creates a polygon with a quadratic extent with the
            edge points being at (0, 0), (0, quad_extent),
            (quad_extent, 0), (quad_extent, quad_extent).
            Also the polygon starts at -100 offset height.

            Args:
                builder: The flatbuffers builder
                quad_extent: The quad_extent on which the other three points
                    are placed

            Returns: The created Polygon2D type type
            """
            polygon_vertices = []
            polygon_vertices.append(fbh.createPoint2d(builder, 0, 0))
            polygon_vertices.append(fbh.createPoint2d(builder, 0, quad_extent))
            polygon_vertices.append(
                fbh.createPoint2d(builder, quad_extent, quad_extent)
            )
            polygon_vertices.append(fbh.createPoint2d(builder, quad_extent, 0))
            return fbh.createPolygon2D(builder, height, -100, polygon_vertices)

        @classmethod
        def time_interval(cls, builder: Builder) -> TimeInterval.TimeInterval:
            timeMin = fbh.createTimeStamp(builder, 1610549273, 0)
            timeMax = fbh.createTimeStamp(builder, 1938549273, 0)
            return fbh.createTimeInterval(builder, timeMin, timeMax)

        @classmethod
        def label_category(
            cls, builder: Builder
        ) -> LabelCategory.LabelCategory:
            category = ["0"]

            labelCategoryMsgs = []
            for cat in category:
                labels = ["testlabel0", "testlabel1"]

                labelMsgs = []
                for label in labels:
                    labelMsgs.append(fbh.create_label(builder, label, 2))

                datumaro_json = "a very valid datumaro json"
                labelCategoryMsgs.append(
                    fbh.create_label_category(
                        builder, labelMsgs, datumaro_json, cat
                    )
                )
            return labelCategoryMsgs

        @classmethod
        def sparql_query(cls, builder: Builder):
            return None

        @classmethod
        def ontology_uri(cls, builder: Builder):
            return None

        @classmethod
        def projectuuid(cls, builder: Builder, channel: Channel) -> List[str]:
            return [fbh.getProject(builder, channel, "testproject")]

        @classmethod
        # returns list of flatbuffered string, those are registered as ints
        def intanceuuid(
            cls, builder: Builder, channel: Channel, proj_uuid: int
        ) -> List[int]:
            query_builder = FbQuery(
                channel, enum_types={EnumFbQuery.PROJECTUUID}
            )
            query_builder.set_active_function(
                EnumFbQuery.PROJECTUUID, lambda: proj_uuid
            )
            query_builder.assemble_datatype_instance()
            query_inst_builder = FbQueryInstance(
                channel, enum_types={EnumFbQueryInstance.QUERY}
            )
            query_inst_builder.set_active_function(
                EnumFbQueryInstance.QUERY,
                lambda: query_builder.datatype_instance,
            )
            query_inst_builder.assemble_datatype_instance()
            query_instance = query_inst_builder.datatype_instance

            serv_man = ServiceManager(channel)
            uuids_pp = serv_man.call_get_instances_fb(builder, query_instance)

            uuids_by_proj = [
                uuids_pp.UuidsPerProject(i)
                for i in range(uuids_pp.UuidsPerProjectLength())
            ]

            uuids = []
            if len(uuids_by_proj) > 0:
                # return every second uuid
                uuids = sorted(
                    [
                        uuids_by_proj[0].Uuids(i).decode()
                        for i in range(0, uuids_by_proj[0].UuidsLength())
                    ]
                )
                uuids = [uuids[i] for i in range(0, len(uuids), 2)]

            return uuids

        @classmethod
        def datauuid(cls, builder: Builder, channel: Channel) -> List[int]:
            query_fb = FbQuery(channel).datatype_instance
            serv_man = ServiceManager(channel)
            images = serv_man.call_get_images_fb(builder, query_fb)
            points = serv_man.call_get_points_fb(builder, query_fb)
            pcl2s = serv_man.call_get_pointcloud2_fb(builder, query_fb)

            image_uuids = sorted(
                [image.Header().UuidMsgs().decode() for image in images]
            )
            point_uuids = sorted(
                [point.Header().UuidMsgs().decode() for point in points]
            )
            pcl2_uuids = sorted(
                [pcl.Header().UuidMsgs().decode() for pcl in pcl2s]
            )

            # debugging
            # print("images: " + str(image_uuids))
            # print("points: " + str(point_uuids))
            # print("pcls: " + str(pcl2_uuids))

            # fill up return list with every second uuid
            ret_lst = [image_uuids[i] for i in range(0, len(image_uuids), 2)]
            ret_lst += [point_uuids[i] for i in range(0, len(point_uuids), 2)]
            ret_lst += [pcl2_uuids[i] for i in range(0, len(pcl2_uuids), 2)]

            return ret_lst

        @classmethod
        def max_num_data(cls, builder: Builder):
            return 2

    class FbDefaults:
        @classmethod
        def query(cls, channel: Channel) -> Query:
            return FbQuery(channel).datatype_instance


# shorten datatype definition
Dtypes = DatatypeImplementations
