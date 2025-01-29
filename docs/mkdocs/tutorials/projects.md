# Creating & Retrieving Projects

## Creating new projects

New projects for new data can be created in the following way:

Source:
[examples/gRPC/meta/gRPC_pb_createProject.py](https://github.com/DFKI-NI/seerep/blob/main/examples/python/gRPC/meta/gRPC_pb_createProject.py)

```python
--8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/examples/python/gRPC/meta/gRPC_pb_createProject.py"
```

Output:

``` bash
The new project on the server is (name/uuid):
        testproject ff739be8-cf0c-4657-bff0-f66f3e7f578d
```

## Retrieving projects

After we created two projects, we can query them. Currently the name doesn't
have to be unique.

<!-- markdownlint-disable -->

=== "Flatbuffers"

    Source:
    [examples/gRPC/meta/gRPC_fb_getProjects.py](https://github.com/DFKI-NI/seerep/blob/main/examples/python/gRPC/meta/gRPC_fb_getProjects.py)


    ```python
    --8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/examples/python/gRPC/meta/gRPC_fb_getProjects.py"
    ```

    Output:

    ```
    The server has the following projects (name/uuid):
        testproject 842de425-2d50-4adf-8aa3-6df257a7c76c
        testproject ff739be8-cf0c-4657-bff0-f66f3e7f578d
    ```

=== "Protocol Buffers"

    Source:
    [examples/gRPC/meta/gRPC_pb_getProjects.py](https://github.com/DFKI-NI/seerep/blob/main/examples/python/gRPC/meta/gRPC_pb_getProjects.py)

    ```python
    --8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/examples/python/gRPC/meta/gRPC_pb_getProjects.py"
    ```

    Output:

    ```
    The server has the following projects (name/uuid):
        testproject 842de425-2d50-4adf-8aa3-6df257a7c76c
        testproject ff739be8-cf0c-4657-bff0-f66f3e7f578d
    ```
