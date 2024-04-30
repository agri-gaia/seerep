# Tests

We are currently working on integrating more tests into SEEREP.
[GoogleTest](https://github.com/google/googletest) for C++ related tests and
[pytest](https://github.com/pytest-dev/pytest) for python related tests are used as testing frameworks.
The tests are run for every PR and push to the main branch in a GitHub workflow,
they can also be run locally in a couple of different ways.

## Types of tests

### C++ related tests

- ROS Conversions ([ros_to_fb_conversion_test.cpp](https://github.com/agri-gaia/seerep/blob/main/seerep_ros/seerep_ros_conversions_fb/test/ros_to_fb_conversion_test.cpp)):
    Tests the SEEREP conversion functions for ros message types to flatbuffers message types and vice versa.
- HDF Interface:
    This tests the SEEREP hdf5 write and read functionality with the [image.fbs](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/image.fbs),
    [image.proto](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/image.proto) datatypes and therefore
    serves as integration tests.
    They are defined in [pb_write_load_test.cpp](https://github.com/agri-gaia/seerep/blob/main/seerep_hdf5/seerep_hdf5_pb/test/pb_write_load_test.cpp),
    [fb_write_load_test.cpp](https://github.com/agri-gaia/seerep/blob/main/seerep_hdf5/seerep_hdf5_fb/test/fb_write_load_test.cpp),
    [py_write_load_test.cpp](https://github.com/agri-gaia/seerep/blob/main/seerep_hdf5/seerep_hdf5_py/test/py_write_load_test.cpp).

### Python related integration tests

- Integration tests:
    These tests are testing most of the python examples which are sending and retrieving SEEREP
    server data utilizing protobuf or flatbuffers as the communication medium.
    The related script files are located in the [tests](https://github.com/agri-gaia/seerep/blob/main/tests) directory.

## Running tests locally

### C++ tests using catkin

The C++ related tests can be run via `catkin` in the command line. When run without a
specific package, all tests in the workspace are executed. But while this is very
convenient, catkin does not provide much information/output if a test fails.

```bash
catkin test <specific-package>
```

### C++ tests using Executables

If you would like to run the tests via their executables, they are located under
`/seerep/devel/bin/<test-name>` or `/seerep/build/<package>/<test-name>`.

### Python integration tests using pytest

The python tests can be executed in `/seerep/src` using the

```bash
pytest
```

command in a shell environment.

To execute a subset of the python tests specify the path to a specific test folder or file as the first argument to the
pytest command, e.g.

```bash
# assuming we're currently in the `/seerep/src` directory
# would run all the tests in the `meta` directory and in all subdirectories of `meta`
pytest tests/python/gRPC/meta

# just runs the tests defined in the `test_gRPC_pb_projectCreation.py` script
pytest tests/python/gRPC/meta/test_gRPC_pb_projectCreation.py
```

An alternative approach is to navigate into the subfolder and execute the `pytest` command without arguments from there,
e.g.

```bash
# assuming we're currently in the `/seere/src/tests/python/gRPC/meta` directory
# just executes the tests in the `meta` directory and all subdirectories of `meta`
pytest
```

### Testing through VSCode

Another way to run the tests is via the VSCode test explorer (triangle test-tube on
the left bar of VSCode). If you have done a fresh installation of the project, it
can happen, that the test cases won't be recognized. In order to fix that, just
restart the development container. For that, you can use `Reopen Folder Locally`
and then `Reopen In Container` again. Now you should be able to see the test cases as,
in the example below:

![vs-code-test-explorer](../../imgs/VSCode-Testing.png)

The icons in the top of the test explorer are mostly self-explanatory, refresh, all tests
can be run, a single test can be debugged, and a terminal can be opened to print
the output of the tests.
