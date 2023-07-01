# test file for
#   gRPC_pb_createProject.py
#   gRPC_pb_getProject.py
# utilizes and assumes the correctness of
#   gRPC.util.fb_helper.createProjectInfo

import flatbuffers
from gRPC.meta import gRPC_pb_createProject as project_creation
from gRPC.meta import gRPC_pb_getProjects as projects_query
from gRPC.util import fb_helper


def test_gRPC_pb_createProjectAndGetProjects(grpc_channel):
    # get already created projects
    current_projects = projects_query.get_projects(grpc_channel)

    # contains tuples consisting of project name and project uuid
    created_projects = []
    # create_project returns Tuple[str, str] = (<project_name>, <project_uuid>)
    created_projects.append(project_creation.create_project(grpc_channel))
    created_projects.append(project_creation.create_project(grpc_channel))

    projects_list = projects_query.get_projects(grpc_channel)

    # check if count of created projects matches the count of the newly queried projects,
    # returned by the gRPC_pb_getProjects example
    assert len(projects_list) - len(current_projects) == len(created_projects)

    for p in created_projects:
        assert p in projects_list

        # teardown
        builder = flatbuffers.Builder(1024)
        fb_helper.deleteProject(grpc_channel, builder, p[0], p[1])
