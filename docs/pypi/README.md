# SEEREP gRPC API

This packages provides the [Protocol Buffers](https://protobuf.dev/) and
[Flatbuffers](https://google.github.io/flatbuffers/) classes for gRPC API of
[SEEREP](https://github.com/agri-gaia/seerep).

## Usage

Import the classes from the `seerep.pb` or `seerep.fb` modules like this:

```python
from seerep.pb import image_pb2 as image
from seerep.pb import image_service_pb2_grpc as imageService
```

```python
from seerep.fb import Image
from seerep.fb import image_service_grpc_fb as imageService
```

## Examples

For more advanved examples take a look in the SEEREP repository
[here](https://github.com/agri-gaia/seerep/tree/main/examples/python/gRPC).
