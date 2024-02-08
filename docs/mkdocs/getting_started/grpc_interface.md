# gRPC Interface

SEEREP uses [gRPC](https://grpc.io/), thus enabling usage across multiple programming languages due to the
language-independent nature of [Protocol Buffers](https://protobuf.dev/) and [Flatbuffers](https://flatbuffers.dev/).
Given the widespread use of Python and C++ in robotics, we offer guidance on getting started with the API in these
languages. For complete examples demonstrating sending and querying data, please refer to the
[examples section](../examples/).

## Python

We offer a [PyPi Package](https://pypi.org/project/seerep-grpc/) containing the generated messages and
services, along with helpful utility scripts. To install the lastest version, use:

```bash
pip install seerep-grpc
```

The package consits of three sub-packages:

```txt
fb/
|-- PointCloud2.py
|-- point_cloud_service_grpc_fb.py
|-- ... other messages and services
pb/
|-- point_cloud_2_pb2.py
|-- point_cloud_service_pb2_grpc.py
|-- ... other messages and services
util/
|-- commpon.py
|-- fb_helper.py
|-- visualizations.py
```

## C++

The C++ message and service interfaces can be generated either by directly invoking the respective Interface
Definition Language (IDL) compilers (`flatc` and `protoc`) or through integration into CMake.

### Compiler Call

Use these tutorials for Protocol Buffers:

- [C++ Generated Code Guide](https://protobuf.dev/reference/cpp/cpp-generated/#invocation)
- [ProtoBuf gRPC Generating Code](https://grpc.io/docs/languages/cpp/basics/#generating-client-and-server-code)

For Flatbuffers the `flatc` compiler should be called like this:

```bash
flatc --cpp --grpc -I [input dir] -o [output dir] *.fbs
```

### CMake

TODO: After cleanup of CMake files.

## Other languages

For other languages, visit the related Protocol Buffers and Flabuffers documentation pages:

- [ProtoBuf Cross Language Compatibility](https://protobuf.dev/overview/#cross-lang)
- [Flatbuffers Platform / Language / Feature Support](https://flatbuffers.dev/flatbuffers_support.html)
