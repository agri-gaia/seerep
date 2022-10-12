# Creating & Retrieving Projects

## Creating new projects

New projects for new data can be created in the following way:

Source: `examples/gRPC/meta/gRPC_pb_createProject.py`

```python
import os
import sys

import meta_operations_pb2_grpc as metaOperations
import projectCreation_pb2

# importing util functions. Assuming that this file is in the parent dir
# https://github.com/agri-gaia/seerep/blob/6c4da5736d4a893228e97b01a9ada18620b1a83f/examples/python/gRPC/util.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

# Default server is localhost !
channel = util.get_gRPC_channel()

# 1. Get gRPC service object
stub = metaOperations.MetaOperationsStub(channel)

# 2. Create the project object inline and send it to the server
response = stub.CreateProject(projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map"))

print("The new project on the server is (name/uuid):")
print("\t" + response.name + " " + response.uuid)
```

Output:

``` bash
The new project on the server is (name/uuid):
        testproject eff47bc9-c39e-430e-8153-88e0eab65768
```

## Retrieving projects

After we created two projects, we can query them. Currently the name doesn't have to be unique.

=== "Protocol Buffers"

    Source: `examples/gRPC/meta/gRPC_pb_getProjects.py`

    ```python
    import os
    import sys

    import meta_operations_pb2_grpc as metaOperations
    from google.protobuf import empty_pb2

    # importing util functions. Assuming that this file is in the parent dir
    # https://github.com/agri-gaia/seerep/blob/6c4da5736d4a893228e97b01a9ada18620b1a83f/examples/python/gRPC/util.py
    script_dir = os.path.dirname(__file__)
    util_dir = os.path.join(script_dir, '..')
    sys.path.append(util_dir)
    import util

    # Default server is localhost !
    channel = util.get_gRPC_channel()

    # 1. Get gRPC service object
    stub = metaOperations.MetaOperationsStub(channel)

    # 2. Get all projects on the server
    response = stub.GetProjects(empty_pb2.Empty())


    print("The server has the following projects (name/uuid):")
    for projectinfo in response.projects:
        print("\t" + projectinfo.name + " " + projectinfo.uuid)

    ```

    Output:

    ```
    The Server has the following projects (name/uuid):
        testproject 5c1ed18e-9180-40e1-a79b-594f8266d898
        testproject 9fc3011f-4a3c-400e-9170-06973a6fb395
    ```

=== "Flatbuffers"

    Source: `examples/gRPC/meta/gRPC_fb_getProjects.py`


    ```python
    import os
    import sys

    import flatbuffers
    from fb import Empty, ProjectInfos
    from fb import meta_operations_grpc_fb as metaOperations

    # importing util functions. Assuming that this file is in the parent dir
    # https://github.com/agri-gaia/seerep/blob/6c4da5736d4a893228e97b01a9ada18620b1a83f/examples/python/gRPC/util.py
    script_dir = os.path.dirname(__file__)
    util_dir = os.path.join(script_dir, '..')
    sys.path.append(util_dir)
    import util

    # Default server is localhost !
    channel = util.get_gRPC_channel()

    # 1. Get gRPC service object
    stub = metaOperations.MetaOperationsStub(channel)

    # Create an empty message
    builder = flatbuffers.Builder(1024)
    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    # 2. Get all projects on the server
    responseBuf = stub.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    print("The server has the following projects (name/uuid):")
    for i in range(response.ProjectsLength()):
        print("\t" + response.Projects(i).Name().decode("utf-8") + " " + response.Projects(i).Uuid().decode("utf-8"))
    ```

    Output:

    ```
    The server has the following projects (name/uuid):
        testproject 5c1ed18e-9180-40e1-a79b-594f8266d898
        testproject 9fc3011f-4a3c-400e-9170-06973a6fb395
    ```
