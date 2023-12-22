# SEEREP gRPC API

For easy communication to the server via Python the needed Python files for the gRPC API are available on
[pypi](https://pypi.org/) in the [seerep-grpc](https://pypi.org/project/seerep-grpc/) project. Install them with:

```bash
pip install seerep-grpc
```

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

For more advanved examples take a look at the [Tutorial](/tutorials/overview/) or at the [SEEREP repository](https://github.com/agri-gaia/seerep/tree/main/examples/python/gRPC).
