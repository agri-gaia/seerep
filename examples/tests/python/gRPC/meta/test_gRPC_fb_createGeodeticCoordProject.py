# test file for
#  gRPC_fb_createGeodeticCoordProject.py
# gRPC_fb_getProjects.py
from typing import List

import fb_to_dict
import flatbuffers
import seerep.util.fb_helper as fb_helper
from gRPC.meta import gRPC_fb_createGeodeticCoordProject as project_creation
from gRPC.meta import gRPC_fb_getProjects as projects_query
from seerep.fb import ProjectInfo as PI


def test_gRPC_fb_createGeoProjectAndGetProjects(grpc_channel):
    # get already created projects
    current_projects = projects_query.get_projects(grpc_channel)

    # contains the flatbuffers buffer objects of the projects
    created_projects: List[PI.ProjectInfo] = []
    # create_project returns Tuple[str, str] = (<project_name>, <project_uuid>)
    created_projects.append(project_creation.create_geo_proj(grpc_channel))
    created_projects.append(project_creation.create_geo_proj(grpc_channel))

    # get a list of all created projects
    projects_list: List[PI.ProjectInfo] = projects_query.get_projects(grpc_channel)

    created_projects_dicts = [
        fb_to_dict.fb_obj_to_dict(p_dict) for p_dict in created_projects
    ]
    queried_projects_dicts = [
        fb_to_dict.fb_obj_to_dict(p_dict) for p_dict in projects_list
    ]

    # check if count of created projects matches the count of the newly queried projects,
    # returned by the gRPC_pb_getProjects example
    assert len(projects_list) - len(current_projects) == len(created_projects)

    # manually check if the project parameters are correct
    for proj_info in created_projects:
        proj_name = proj_info.Name().decode("utf-8")
        proj_uuid = proj_info.Uuid().decode("utf-8")
        assert proj_name == "geodeticProject"
        assert proj_uuid != ""
        assert proj_info.Frameid().decode("utf-8") == "2"
        assert (
            proj_info.GeodeticPosition().CoordinateSystem().decode("utf-8")
            == "EPSG::4326"
        )
        # missing
        # assert proj_info.Datum().decode("utf-8") == "EPSG::7030"
        assert proj_info.GeodeticPosition().Altitude() == 4.0
        assert proj_info.GeodeticPosition().Latitude() == 6.0
        assert proj_info.GeodeticPosition().Longitude() == 7.0
        assert proj_info.Version() is not None

    for proj_info in created_projects_dicts:
        assert proj_info in queried_projects_dicts

        # teardown
        builder = flatbuffers.Builder(1024)
        fb_helper.deleteProject(
            grpc_channel, builder, proj_info["Name"], proj_info["Uuid"]
        )
