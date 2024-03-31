# Tests

We are currently working on integrating more tests into SEEREP.
[GoogleTest](https://github.com/google/googletest) for C++ related tests and
[pytest](https://github.com/pytest-dev/pytest) for python related tests are used as testing frameworks.
The tests are run for every PR and push to the main branch in a GitHub workflow,
the tests can also be run locally in a couple of different ways.

## Types of tests

### C++ related tests

- ROS Conversions ([ros_to_fb_conversion_test.cpp](../../../../seerep_ros/seerep_ros_conversions_fb/test/ros_to_fb_conversion_test.cpp)):
    Tests the SEEREP conversion functions for ros message types to flatbuffers message types and vice versa.
- HDF Interface:
    The tests are defined in [pb_write_load_test.cpp](../../../../seerep_hdf5/seerep_hdf5_pb/test/pb_write_load_test.cpp),
    [fb_write_load_test.cpp](../../../../seerep_hdf5/seerep_hdf5_pb/test/fb_write_load_test.cpp),
    [py_write_load_test.cpp](../../../../seerep_hdf5/seerep_hdf5_py/test/py_write_load_test.cpp).
    Those tests test the SEEREP hdf5 write and read functionality with the [image.fbs](../../../../seerep_msgs/fbs/image.fbs),
    [image.proto](../../../../seerep_msgs/protos/image.proto) datatypes and therefore are serving as integration tests.

### Python related integration tests

- Integration tests:
    The tests are located in the [tests](../../../../tests) directory.
    These tests are testing most of the python examples which are sending and retrieving the SEEREP
    server data utilizing protobuf or flatbuffers as the communication medium.

## Running tests locally

### C++ tests using catkin

The C++ related tests can be run via `catkin` in the command line. When run without a
specific package, all tests in the workspace are executed. But while this is very
convenient, catkin does not provide much information/output if a test fails.

```bash
catkin test (<specific-package>)
```

### Vs-Code

Another way to run the C++ test is via the Vs-Code test explorer (triangle test-tube on
the left bar of VS-Code). If you have done a fresh installation of the project, it
can happen, that the test cases won't be recognized. In order to fix that, just
restart the development container. For that, you can use `Reopen Folder Locally`
and then `Reopen In Container` again. Now you should be able to see the test cases as,
in the example below:

![vs-code-test-explorer](../imgs/Vs-Code-Testing.png)

The icons in the top of the test explorer are mostly self-explanatory, refresh, all tests
can be run, a single test can be debugged, and a terminal can be opened to print
the output of the tests.

### Executables

If you would like to run the tests via their executables, they are located under
`/seerep/devel/bin/<test-name>` or `/seerep/build/<package>/<test-name>`.
