# test file for
# gRPC_fb_createGeodeticCoordProject.py
# requires:
# gRPC_fb_getProjects.py
import flatbuffers
from boltons.iterutils import remap
from gRPC.meta import gRPC_fb_createGeodeticCoordProject as project_creation
from gRPC.meta import gRPC_fb_getProjects as projects_query
from seerep.util import common, fb_helper
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def test_gRPC_fb_createGeoProjectAndGetProjects(grpc_channel):
    # get already created projects
    current_projects = remap(
        fb_flatc_dict(projects_query.get_projects(grpc_channel), SchemaFileNames.PROJECT_INFOS)["projects"],
        visit=common.remap_to_snake_case,
    )

    # create_project returns Tuple[str, str] = (<project_name>, <project_uuid>)
    created_project = remap(
        fb_flatc_dict(project_creation.create_geo_proj_raw(grpc_channel), SchemaFileNames.PROJECT_INFO),
        visit=common.remap_to_snake_case,
    )

    # get a list of all created projects
    projects_list = remap(
        fb_flatc_dict(projects_query.get_projects(grpc_channel), SchemaFileNames.PROJECT_INFOS)["projects"],
        visit=common.remap_to_snake_case,
    )

    # check if count of created projects matches the count of the newly queried projects,
    # returned by the gRPC_pb_getProjects example
    assert len(projects_list) - len(current_projects) == 1

    # manually check if the project parameters are correct
    assert created_project["name"] == "geodeticProject"
    assert created_project["uuid"] != ""
    assert created_project["frameid"] == "2"

    assert created_project["geodetic_position"]["coordinate_system"] == "EPSG::4326"
    assert created_project["geodetic_position"]["altitude"] == 4.0
    assert created_project["geodetic_position"]["latitude"] == 6.0
    assert created_project["geodetic_position"]["longitude"] == 7.0
    assert created_project["version"] is not None

    assert created_project in projects_list

    # teardown
    builder = flatbuffers.Builder(1024)
    fb_helper.deleteProject(grpc_channel, builder, created_project["name"], created_project["uuid"])
