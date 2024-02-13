from typing import Generator, Tuple

import cleanup_util as th
import flatbuffers
import pytest
from gRPC.meta import gRPC_pb_createProject as proj_creation
from seerep.util import fb_helper
from seerep.util.common import get_gRPC_channel

# tests are only working with a seerep server listening on the port specified in ./seerep.cfg

# pytest starts relative to where it finds the first pyproject.toml
# in this case it is in the examples folder
def pytest_addoption(parser):
    parser.addoption(
        '--ignore-leftovers',
        action='store_true',
        default=False,
        help='whether pytest should ignore leftovers in the test directory, else the user will be prompted for removal',
    )


@pytest.fixture(scope="session")
def setup_test_struct():
    yield th.setup_test_struct()


# makes sure that all projects and log files in the test folder are deleted
# otherwise if the server failed to terminate correctly, leftover projects could lead to problems
@pytest.fixture(scope="session", autouse=True)
def cleanup_remnants(request) -> None:
    if request.config.getoption("--ignore-leftovers") == True:
        return
    # a workaround to enable stdin while using pytest
    capture_man = request.config.pluginmanager.getplugin("capturemanager")
    capture_man.suspend_global_capture(in_=True)

    th.cleanup_test_remnants()

    capture_man.resume_global_capture()


# provide a instance of the grpc_channel to the tests
@pytest.fixture(scope="session")
def grpc_channel(setup_test_struct):
    channel = get_gRPC_channel(f"localhost:{setup_test_struct[0]}")
    yield channel


# this fixture depends on working project creation and deletion
@pytest.fixture
def project_setup(grpc_channel) -> Generator[Tuple[str, str], None, None]:
    # setup
    proj_name, proj_uuid = proj_creation.create_project(grpc_channel)
    yield proj_name, proj_uuid

    # teardown
    builder = flatbuffers.Builder()
    fb_helper.deleteProject(grpc_channel, builder, proj_name, proj_uuid)


# creates a project with geodetic coordinates
@pytest.fixture
def geo_project_setup(grpc_channel) -> Generator[Tuple[str, str], None, None]:
    # setup
    builder = flatbuffers.Builder()

    # should correspondend to topocentric coordinates:
    # X: 3876524.5192
    # Y: 548466.2095
    # Z: 5018349.9785
    proj_info = fb_helper.createProjectRaw(
        grpc_channel,
        builder,
        "testproject",
        "2",
        "EPSG::4326",
        93,
        52.2264,
        8.0530,
    )

    proj_name, proj_uuid = proj_creation.create_project(grpc_channel)
    yield proj_name, proj_uuid

    # teardown
    fb_helper.deleteProject(grpc_channel, builder, proj_name, proj_uuid)
