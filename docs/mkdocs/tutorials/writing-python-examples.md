# Writing examples for the SEEREP API

To write a example it is a good idea to refer to already written ones. The focus
in this documentation lies on flatbuffers type messages, because of the potential
deprecation of protobuf in the project. All protobuf functionality can be
replicated using flatbuffers, and flatbuffers should be used instead.

In this case the example
[gRPC_fb_addBoundingBox.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py)
will be reviewed. Service type definitions for all available flatbuffers type
services can be found
[here](https://github.com/agri-gaia/seerep/tree/main/seerep_com/fbs).
Type definitions of all flatbuffers types can be found
[here](https://github.com/agri-gaia/seerep/tree/main/seerep_msgs/fbs).

## The code

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py:8:30"
```

First some of the modules to interact with the servers services will be highlighted.
`seerep.fb` contains all python interfaces for the SEEREP services as well as the
Message types. [sereep.util.fb_helper](../reference/python-helpers.md) contains
helper functions related to flatbuffers, for instance functions to create a
message type directly.

### Interaction with SEEREP services and handling the data

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py:32:34"
```

The interaction functionality is contained within this function. With the
function definition, an option should be given to specify a target project, if
the message type allows setting it. Additionally the `grpc_channel` should be a
parameter in order to be able to target servers other than `localhost:9090`.
Both options are useful for testing later. More parameters can be added
optionally, if needed for the test cases.

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py:35:53"
```

At first, if `target_proj_uuid` is not set, the `MetaOperationsStub` utilizing
flatbuffers gRPC communication with the SEEREP server is used to retrieve a list
of all available projects of that server (specifically in the form of
[project_infos.fbs](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/project_infos.fbs)
) and `target_proj_uuid` is set to the uuid of the first project with the name
`testproject` on that list.

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py:54:64"
```

Following on the code requests all images from the project with the `uuid` of
`target_proj_uuid` using the `ImageServiceStub`. The service definition looks as
follows:

```fbs
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/seerep_com/fbs/image_service.fbs"
```

`GetImage()` takes a argument of type
[seerep.fb.Query](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/query.fbs),
a more generic build query type for use in various services in SEEREP, in it's
serialized form and returns data of type
[seerep.fb.Image](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/image.fbs)
from the server.

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py:66:111"
```

This code builds a list of BoundingBoxes adding some sample data into the
components of each BoundingBox. At the beginning two lists are defined `msgToSend`
is a list containing the serialized BoundingBoxes and `bb_list` is a list
containing mappings where each image uuid is mapped to it's added BoundingBoxes.
After that the returned images from the query before are iterated. Next
BoundingBoxes are created and their joint `header` uuids are set to the
appropriate `project_uuid` and `msg_uuid` to match that specific image. At the
end the BoundingBoxes are serialized and added to the lists.

The type definition of
[BoundingBoxes2DLabeledStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingboxes2d_labeled_stamped.fbs)
looks as follows:

```fbs
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/seerep_msgs/fbs/boundingboxes2d_labeled_stamped.fbs"
```

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py:113:114"
```

Lastly the service is called, the BoundingBoxes are send to the SEEREP server
and the list with the mappings is returned for further use. Note that the
flatbuffers objects are not returned in their deserialized state as the function
`fb_flatc_dict` defined in
[here](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/util/fb_to_dict.py)
makes use of that state.

### Wrapping the raw function

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py:116:123"
```

This function is essentially just a wrapper for `add_bb_raw()` to return the
deserialized objects to be accessed through their regular flatbuffers interfaces
(in this case of type
`BoundingBoxes2DLabeledStamped.BoundingBoxes2DLabeledStamped`).

### Allow for independent execution of the script

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_addBoundingBox.py:126:141"
```

The last part can execute the script independently and targets the server at the
default address, which is `localhost:9090`. On successful execution a subset of
the sent data based on the returned mapping is printed.

## Some important considerations

To conclude for most of the examples it is best practice to follow this structure,
namely first having a function which returns the serialized data, then wrapping
that function to return the deserialized variant and at the end the
`if __name__ == "__main__"` part of the script, such that the script can be
executed independently. Of course functionality can be outsourced into other
functions, when it makes sense. This structure eases the process of writing
tests later on (see [writing-tests.md](writing-python-tests.md)), especially
when `fb_flatc_dict` should be utilized.
