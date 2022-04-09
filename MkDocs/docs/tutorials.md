# Tutorials

This tutorial will give an introduction on how to use SEEREP. The following
topics are covered:

- Creating and retrieving projects
- Sending and querying data with gRPC
- Sending ROS messages to SEEREP

Before running any of the code exampels, **make sure that the SEEREP server is
running**.

## GRPC

GRPC is used to send data back and forth between the robot and the cloud
sever-cluster. In the build process, the protoc compiler created python classes
from the service and messages definitions in protocol buffer (.proto) files. The
generated classes are now used in the examples, for more information on the
process, visit the
gRPC [documentation](https://grpc.io/docs/languages/python/quickstart/#generate-grpc-code).

### Creating new projects

1. Import gRPC and necessary service and message modules.
2. Create a server connection, since `localhost` is used, we don't need any
   encryption.
3. Create a stub (client) which provides the methods from the remote server.
4. Call the `CreateProject`method with a `projectCreation` message as the
   parameter.

```python
import grpc
import meta_operations_pb2_grpc as metaOperations # project service
import projectCreation_pb2 # project creation message

server = "localhost:9090"
channel = grpc.insecure_channel(server)

stub = metaOperations.MetaOperationsStub(channel)

# create new project
response = stub.CreateProject(
    projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map")
)

print(f"The new project on the server {server} is (name/uuid):")
print(f"\t {response.name} {response.uuid}")
```

### Retrieving new projects

Retrieving projects follows almost the same steps as creating a project, except
that the `GetProjects` method as no parameter and needs an `empty message`.

```python
import grpc
import meta_operations_pb2_grpc as metaOperations # project service
from google.protobuf import empty_pb2 # empty message

server = "localhost:9090"
channel = grpc.insecure_channel(server)

stub = metaOperations.MetaOperationsStub(channel)

response = stub.GetProjects(empty_pb2.Empty())

print(f"{server} has the following projects (name/uuid):")
for projectinfo in response.projects:
    print(f"\t {projectinfo.name} {projectinfo.uuid}")
```

### Sending images

1. Import gRPC and necessary service and message modules.
2. Check if we have an existing project, if not create one.
3. Create an image to send. Or transform an image into the necessary structure.
4. Set meta information for the image.
5. Write image to the server.

```python
import image_pb2 as image # image message
import image_service_pb2_grpc as imageService # image service
import projectCreation_pb2 as projectCreation # project creation message
import meta_operations_pb2_grpc as metaOperations # project service
from google.protobuf import empty_pb2 # empty message
import grpc
import time
import numpy as np

channel = grpc.insecure_channel("localhost:9090")

stubImage = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

# check if the test-project is already present
found = False
for project in response.projects:
    print(f"{project.name} {project.uuid}")
    if project.name == "testproject":
        projectuuid = project.uuid
        found = True

# create a project if the test-project is not present
if not found:
    creation = projectCreation.ProjectCreation(name="testproject", mapFrameId="map")
    response = stubMeta.CreateProject(creation)
    projectuuid = response.uuid

theImage = image.Image()
theTime = time.time()

# create 10 images
for n in range(10):
    theImage = image.Image()

    rgb = []
    lim = 256 # 256x256 image
    for i in range(lim):
        for j in range(lim):
            x = float(i) / lim
            y = float(j) / lim
            z = float(j) / lim
            r = np.ubyte((x * 255.0 + n) % 255)
            g = np.ubyte((y * 255.0 + n) % 255)
            b = np.ubyte((z * 255.0 + n) % 255)
            # print(r, g, b)
            rgb.append(r)
            rgb.append(g)
            rgb.append(b)

    theImage.header.frame_id = "camera"
    theImage.header.stamp.seconds = theTime + n
    theImage.header.stamp.nanos = 0
    theImage.header.uuid_project = projectuuid
    theImage.height = lim
    theImage.width = lim
    theImage.encoding = "rgb8"
    theImage.step = 3 * lim
    theImage.data = bytes(rgb)

    # add labels to the images
    for i in range(0, 10):
        theImage.labels_general.append("testlabelgeneral" + str(i))

    uuidImg = stub.TransferImage(theImage)

    print("uuid of transfered img: " + uuidImg.message)
```

### Querry images

1. Import gRPC and necessary service and message modules.
2. Get the UUID of the project you want to query.
3. Create a query and set the query parameters.
4. Get back images which match the query parameters.

```python
import image_service_pb2_grpc as imageService  # image service
import meta_operations_pb2_grpc as metaOperations # project service
import query_pb2 as query # query message
from google.protobuf import empty_pb2 # empty message
import grpc

channel = grpc.insecure_channel("localhost:9090")

stub = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

# get uuid of the testproject
projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "testproject":
        projectuuid = project.uuid

# if the test project doesn't exits stop
if projectuuid == "":
    sys.exit()

# cerate and query and set query parameters
theQuery = query.Query()
theQuery.projectuuid = projectuuid

# since epoche
theQuery.timeinterval.time_min = 1638549273
theQuery.timeinterval.time_max = 1938549273

# labels
theQuery.label.extend(["testlabel0"])


for img in stub.GetImage(theQuery):
    print("uuid of transfered img: " + img.labels_general[0])

```
