# Creating & Retrieving Projects

## Creating new projects

New projects for new data can be created in the following way:

Source: [examples/gRPC/meta/gRPC_pb_createProject.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/meta/gRPC_pb_createProject.py)

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/meta/gRPC_pb_createProject.py"
```

Output:

``` bash
The new project on the server is (name/uuid):
        testproject eff47bc9-c39e-430e-8153-88e0eab65768
```

## Retrieving projects

After we created two projects, we can query them. Currently the name doesn't have to be unique.

<!-- markdownlint-disable -->

=== "Flatbuffers"

    Source: [examples/gRPC/meta/gRPC_fb_getProjects.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/meta/gRPC_fb_getProjects.py)


    ```python
    --8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/meta/gRPC_fb_getProjects.py"
    ```

    Output:

    ```
    The server has the following projects (name/uuid):
        testproject 5c1ed18e-9180-40e1-a79b-594f8266d898
        testproject 9fc3011f-4a3c-400e-9170-06973a6fb395
    ```

=== "Protocol Buffers"

    Source: [examples/gRPC/meta/gRPC_pb_getProjects.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/meta/gRPC_pb_getProjects.py)

    ```python
    --8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/meta/gRPC_pb_getProjects.py"
    ```

    Output:

    ```
    The Server has the following projects (name/uuid):
        testproject 5c1ed18e-9180-40e1-a79b-594f8266d898
        testproject 9fc3011f-4a3c-400e-9170-06973a6fb395
    ```
