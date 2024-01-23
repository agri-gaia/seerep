# gRPC Interface

SEEREP's API uses gRPC, thus enabling compatibility across multiple programming languages due to the
language-independent nature of Protocol Buffers and Flatbuffers. Given the widespread use of Python and C++ in robotics,
we offer guidance on getting started with the API. For complete examples demonstrating send and query functionalities,
please refer to the [examples section](../examples/).

## Python

We offer a [PyPi package](https://pypi.org/project/seerep-grpc/) for Python containing the generated messages and
services, along with helpful scripts. To install the package, use the following:

```bash
pip install seerep-grpc
```

The package consits of three directories:

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
|-- commpon.py //
|-- fb_helper.py
|-- visualizations.py
```

## C++

The C++ messages and service interfaces can be generated either by directly invoking the respective Interface
Definition Language (IDL) compilers (flatc and protoc) or through integration into CMake.

## Compiler Call

Use these helpful tutorials for Protocol Buffers:

- [C++ Generated Code Guide](https://protobuf.dev/reference/cpp/cpp-generated/#invocation)
- [ProtoBuf gRPC Generating Code](https://grpc.io/docs/languages/cpp/basics/#generating-client-and-server-code)

For Flatbuffers the `flatc` compiler should be called like this:

```bash
flatc --cpp --grpc -I [input dir] -o [output dir] *.fbs
```

## CMake

TODO: After cleanup of CMake Files.

## Other languages

For other languages, visit the related Protocol Buffers and Flatubbers documentation pages:

- [ProtoBuf Cross Language Compatibility](https://protobuf.dev/overview/#cross-lang)
- [Flatbuffers Platform / Language / Feature Support](https://flatbuffers.dev/flatbuffers_support.html)
