import flatbuffers
from gRPC.meta import gRPC_pb_createProject as project_creation
from gRPC.meta import gRPC_pb_getProjects as projects_query
from seerep.fb import ProjectInfo
from seerep.fb import meta_operations_grpc_fb as meta_ops
from seerep.util.common import get_gRPC_channel


def test_gRPC_pb_createProject_getProjects():
    # contains tuples consisting of project name and project uuid
    created_projects = []
    created_projects.append(project_creation.create_project())
    created_projects.append(project_creation.create_project())

    projects_list = projects_query.get_projects()

    # check if count of created projects matches the count of existant projects,
    # returned by the gRPC_pb_getProjects example
    assert len(projects_list) == len(created_projects)
    for p in created_projects:
        assert p in projects_list

        # teardown
        # builder = flatbuffers.Builder(1024)
        # stub = meta_ops.MetaOperationsStub(get_gRPC_channel())
        # stub.DeleteProject()
