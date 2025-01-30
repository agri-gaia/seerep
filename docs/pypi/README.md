# SEEREP gRPC API

This packages provides the [Protocol Buffers](https://protobuf.dev/) and
[Flatbuffers](https://google.github.io/flatbuffers/) classes for gRPC API of
[SEEREP](https://github.com/DFKI-NI/seerep).

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
[here](https://github.com/DFKI-NI/seerep/tree/main/examples/python/gRPC).

## Note

If you are not using the wheel, you manually need to install the `flatc` compiler
since it's not available on pypi. Instructions can be found
[here](https://github.com/DFKI-NI/seerep/blob/e9872c51fe6343984ff47ccba29f064774da7296/docker/base/Dockerfile#L93-L101)
