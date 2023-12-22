# Sending & Querying images

## Sending images

In this example we want to send images with labeled bounding boxes as well as general labels to SEEREP.

In order to save images, we need to mandatorily provide the intrinsics of the camera used to capture them.
After the sucessful save of camera intrinsics, we need to provide the uuid of it along with the images.
SEEREP will ensure that the Camera Intinsics UUID provided with an image has a UUID stored against it.

Additionally we add some coordinate transformations at the end.

Source: `examples/python/gRPC/images/gRPC_pb_sendLabeledImage.py`

```python
#!/usr/bin/env python3

import os
import sys
import time
import uuid

import numpy as np
from google.protobuf import empty_pb2
from seerep.pb import boundingbox2d_labeled_pb2 as boundingbox2d_labeled
from seerep.pb import (
    boundingbox2d_labeled_with_category_pb2 as boundingbox2d_labeled_with_category,
)
from seerep.pb import camera_intrinsics_pb2 as cameraintrinsics
from seerep.pb import camera_intrinsics_service_pb2_grpc as camintrinsics_service
from seerep.pb import image_pb2 as image
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_with_instance_pb2 as labelWithInstance
from seerep.pb import (
    labels_with_instance_with_category_pb2 as labels_with_instance_with_category,
)
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2 as projectCreation
from seerep.pb import tf_service_pb2_grpc as tfService
from seerep.pb import transform_stamped_pb2 as tf
from seerep.util.common import get_gRPC_channel

# Default server is localhost !
channel = get_gRPC_channel(target="local")

# 1. Get gRPC service objects
stub = imageService.ImageServiceStub(channel)
stubTf = tfService.TfServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)
stubCI = camintrinsics_service.CameraIntrinsicsServiceStub(channel)

# 2. Get all projects from the server
response = stubMeta.GetProjects(empty_pb2.Empty())

# 3. Check if we have an existing test project, if not, one is created.
found = False
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "testproject":
        projectuuid = project.uuid
        found = True

if not found:
    creation = projectCreation.ProjectCreation(name="testproject", mapFrameId="map")
    projectCreated = stubMeta.CreateProject(creation)
    projectuuid = projectCreated.uuid

theTime = int(time.time())

#####
# A valid camera intrinsics UUID is needed here for succesful storage of Images
# Add new Camera Intrinsics

ciuuid = str(uuid.uuid4())
print("Camera Intrinsics will be saved against the uuid: ", ciuuid)

camin = cameraintrinsics.CameraIntrinsics()

camin.header.stamp.seconds = 4
camin.header.stamp.nanos = 3

camin.header.frame_id = "camintrinsics"

camin.header.uuid_project = projectuuid
camin.header.uuid_msgs = ciuuid

camin.region_of_interest.x_offset = 2
camin.region_of_interest.y_offset = 1
camin.region_of_interest.height = 5
camin.region_of_interest.width = 4
camin.region_of_interest.do_rectify = 4

camin.height = 5
camin.width = 4

camin.distortion_model = "plump_bob"

camin.distortion.extend([3, 4, 5])

camin.intrinsic_matrix.extend([3, 4, 5])
camin.rectification_matrix.extend([3, 4, 5])
camin.projection_matrix.extend([3, 4, 5])

camin.binning_x = 6
camin.binning_y = 7

stubCI.TransferCameraIntrinsics(camin)

# 4. Create ten images
for n in range(10):
    theImage = image.Image()

    rgb = []
    lim = 256  # 256 x 256 pixels
    for i in range(lim):
        for j in range(lim):
            x = float(i) / lim
            y = float(j) / lim
            z = float(j) / lim
            r = np.ubyte((x * 255.0 + n) % 255)
            g = np.ubyte((y * 255.0 + n) % 255)
            b = np.ubyte((z * 255.0 + n) % 255)
            rgb.append(r)
            rgb.append(g)
            rgb.append(b)

    # Add image meta-data
    theImage.header.frame_id = "camera"
    theImage.header.stamp.seconds = theTime + n
    theImage.header.stamp.nanos = 0
    theImage.header.uuid_project = projectuuid
    theImage.height = lim
    theImage.width = lim
    theImage.encoding = "rgb8"
    theImage.step = 3 * lim
    theImage.uuid_camera_intrinsics = ciuuid

    # Add image data
    theImage.data = bytes(rgb)

    for iCategory in range(0, 2):
        bbCat = boundingbox2d_labeled_with_category.BoundingBox2DLabeledWithCategory()
        bbCat.category = str(iCategory)
        # 5. Create bounding boxes with labels
        bb = boundingbox2d_labeled.BoundingBox2DLabeled()
        for i in range(0, 2):
            bb.labelWithInstance.label.label = "testlabel" + str(i)
            bb.labelWithInstance.label.confidence = i / 10.0
            bb.labelWithInstance.instanceUuid = str(uuid.uuid4())
            bb.boundingBox.center_point.x = 0.01 + i / 10
            bb.boundingBox.center_point.y = 0.02 + i / 10
            bb.boundingBox.spatial_extent.x = 0.03 + i / 10
            bb.boundingBox.spatial_extent.y = 0.04 + i / 10
            bbCat.boundingBox2DLabeled.append(bb)
        theImage.labels_bb.append(bbCat)

        # # 6. Add general labels to the image
        labelsCat = labels_with_instance_with_category.LabelsWithInstanceWithCategory()
        labelsCat.category = str(iCategory)
        for i in range(0, 2):
            label = labelWithInstance.LabelWithInstance()
            label.label.label = "testlabelgeneral" + str(i)
            label.label.confidence = i / 10.0
            # assuming that that the general labels are not instance related -> no instance uuid
            # label.instanceUuid = str(uuid.uuid4())
            labelsCat.labelWithInstance.append(label)
        theImage.labels_general.append(labelsCat)

    # 7. Send image to the server
    uuidImg = stub.TransferImage(theImage)

    print("uuid of transfered img: " + uuidImg.message)

# 8. Add coordinate transformations and send them to the server
theTf = tf.TransformStamped()
theTf.header.frame_id = "map"
theTf.header.stamp.seconds = theTime
theTf.header.uuid_project = projectuuid
theTf.child_frame_id = "camera"
theTf.transform.translation.x = 1
theTf.transform.translation.y = 2
theTf.transform.translation.z = 3
theTf.transform.rotation.x = 0
theTf.transform.rotation.y = 0
theTf.transform.rotation.z = 0
theTf.transform.rotation.w = 1
stubTf.TransferTransformStamped(theTf)

theTf.header.stamp.seconds = theTime + 10
theTf.transform.translation.x = 100
theTf.transform.translation.y = 200
theTf.transform.translation.z = 300
stubTf.TransferTransformStamped(theTf)

```

Output:

```bash
uuid of transfered img: 5836f989-adbb-46a0-a689-9e0527d457fe
uuid of transfered img: 5b39487d-238a-43e4-a8a4-0f7efb876e8b
uuid of transfered img: 00ced216-40b1-4d54-817f-11c413b228c6
uuid of transfered img: 5d330208-e534-4bfb-b242-00295e6b027d
uuid of transfered img: 18f00f33-0ac5-4221-a428-13e2c37b27cd
uuid of transfered img: ed5134c8-da18-476f-9c1b-60cea04c5627
uuid of transfered img: 22c995b0-c2b6-4c94-85a3-b1118ae3ac7d
uuid of transfered img: 9d980608-85ff-4e62-a6be-01c05bd59de9
uuid of transfered img: 24204e3f-1cbd-4cab-99b6-803b3944b450
uuid of transfered img: 797f2e1a-f9e8-4ed3-94b3-a5d101aa0697
```

## Query images

Now we want to query the previously send images with some criteria. Possible query parameters are:

* 2D Polygon (spatial query)

Spatial queries in SEEREP are performed using a 2D polygon. This polygon should be simple (no more than 2
vertices on the same edge) and convex (no edges curving inward). This 2D polygon has a location on the z axis,
and a height. Queries are performed by forming an encompassing axis aligned bounding box out of the polygon.
This can lead to an AABB larger than the polygon. This poses the potential problem of returning results to the
user which are not fully inside the polygon from the query. But it is inside the AABB derived from it for the
purpose of performing the query. This problem is resolved by providing a boolean variable called
fullyEncapsulated`. If false, resultant polygons, which are partially inside the query polygon are also returned.

* A time interval (temporal query)

* Labels (semantic query)

<!-- markdownlint-disable -->

=== "Protocol Buffers"

    Source: `examples/python/gRPC/images/gRPC_pb_queryImage.py`

    ```python
    #!/usr/bin/env python3

    import sys

    from google.protobuf import empty_pb2
    from seerep.pb import image_service_pb2_grpc as imageService
    from seerep.pb import label_pb2
    from seerep.pb import labels_with_category_pb2 as labels_with_category
    from seerep.pb import meta_operations_pb2_grpc as metaOperations
    from seerep.pb import query_pb2 as query
    from seerep.pb import point2d_pb2 as point2d
    from seerep.util.common import get_gRPC_channel

    # Default server is localhost !
    channel = get_gRPC_channel()

    # 1. Get gRPC service objects
    stub = imageService.ImageServiceStub(channel)
    stubMeta = metaOperations.MetaOperationsStub(channel)

    # 2. Get all projects from the server
    response = stubMeta.GetProjects(empty_pb2.Empty())

    # 3. Check if we have an existing test project, if not, we stop here
    projectuuid = ""
    for project in response.projects:
        print(project.name + " " + project.uuid + "\n")
        if project.name == "testproject":
            projectuuid = project.uuid

    if projectuuid == "":
        sys.exit()

    # 4. Create a query with parameters
    theQuery = query.Query()
    theQuery.projectuuid.append(projectuuid)

    theQuery.polygon.z = -1
    theQuery.polygon.height = 7

    l = 100
    bottom_left = point2d.Point2D()
    bottom_left.x = -l
    bottom_left.y = -l
    theQuery.polygon.vertices.append(bottom_left)

    top_left = point2d.Point2D()
    top_left.x = -l
    top_left.y = l
    theQuery.polygon.vertices.append(top_left)

    top_right = point2d.Point2D()
    top_right.x = l
    top_right.y = l
    theQuery.polygon.vertices.append(top_right)

    bottom_right = point2d.Point2D()
    bottom_right.x = l
    bottom_right.y = -l
    theQuery.polygon.vertices.append(bottom_right)

    # since epoche
    theQuery.timeinterval.time_min.seconds = 1638549273
    theQuery.timeinterval.time_min.nanos = 0
    theQuery.timeinterval.time_max.seconds = 1938549273
    theQuery.timeinterval.time_max.nanos = 0

    # labels
    label = labels_with_category.LabelsWithCategory()
    label.category = "0"
    labelWithConfidence = label_pb2.Label()
    labelWithConfidence.label = "testlabel0"
    label.labels.extend([labelWithConfidence])
    theQuery.labelsWithCategory.append(label)

    # 5. Query the server for images matching the query and iterate over them
    for img in stub.GetImage(theQuery):
        print(
            f"uuidmsg: {img.header.uuid_msgs}"
            + "\n"
            + f"first label: {img.labels_bb[0].boundingBox2DLabeled[0].labelWithInstance.label.label}"
            + "\n"
            + f"first label confidence: {img.labels_bb[0].boundingBox2DLabeled[0].labelWithInstance.label.confidence}"
            + "\n"
            + "First bounding box (Xcenter,Ycenter,Xextent,Yextent):"
            + " "
            + str(img.labels_bb[0].boundingBox2DLabeled[0].boundingBox.center_point.x)
            + " "
            + str(img.labels_bb[0].boundingBox2DLabeled[0].boundingBox.center_point.y)
            + " "
            + str(img.labels_bb[0].boundingBox2DLabeled[0].boundingBox.spatial_extent.x)
            + " "
            + str(img.labels_bb[0].boundingBox2DLabeled[0].boundingBox.spatial_extent.y)
            + "\n"
        )
    ```

    Output:

    ```
    testproject 3af70ba8-1e81-4f60-86d2-a4257d88f01e

    first label: testlabel0
    First bounding box (Xmin, Ymin, Xmax, Ymax): 0.01 0.02 0.03 0.04

    first label: testlabel0
    First bounding box (Xmin, Ymin, Xmax, Ymax): 0.01 0.02 0.03 0.04

    first label: testlabel0
    First bounding box (Xmin, Ymin, Xmax, Ymax): 0.01 0.02 0.03 0.04

    first label: testlabel0
    First bounding box (Xmin, Ymin, Xmax, Ymax): 0.01 0.02 0.03 0.04
    ```

=== "Flatbuffers"

    Source: `examples/images/gRPC/images/gRPC_fb_queryImage.py`

    ```python
    #!/usr/bin/env python3

    import flatbuffers
    from seerep.fb import Image
    from seerep.fb import image_service_grpc_fb as imageService
    from seerep.util.common import get_gRPC_channel
    from seerep.util.fb_helper import (
        createLabelWithCategory,
        createLabelWithConfidence,
        createPoint2d,
        createPolygon2D,
        createQuery,
        createTimeInterval,
        createTimeStamp,
        getProject,
    )

    builder = flatbuffers.Builder(1024)
    # Default server is localhost !
    channel = get_gRPC_channel()

    # 1. Get all projects from the server
    projectuuid = getProject(builder, channel, 'testproject')

    # 2. Check if the defined project exist; if not exit
    if not projectuuid:
        print("project doesn't exist!")
        exit()

    # 3. Get gRPC service object
    stub = imageService.ImageServiceStub(channel)


    # Create all necessary objects for the query
    l = 100
    polygon_vertices = []
    polygon_vertices.append(createPoint2d(builder, -1.0 * l, -1.0 * l))
    polygon_vertices.append(createPoint2d(builder, -1.0 * l, l))
    polygon_vertices.append(createPoint2d(builder, l, l))
    polygon_vertices.append(createPoint2d(builder, l, -1.0 * l))
    polygon2d = createPolygon2D(builder, 7, -1, polygon_vertices)

    timeMin = createTimeStamp(builder, 1610549273, 0)
    timeMax = createTimeStamp(builder, 1938549273, 0)
    timeInterval = createTimeInterval(builder, timeMin, timeMax)

    projectUuids = [builder.CreateString(projectuuid)]
    # list of categories
    category = ["0"]
    # list of labels per category
    labels = [
        [
            createLabelWithConfidence(builder, "testlabel0"),
            createLabelWithConfidence(builder, "testlabelgeneral0"),
        ]
    ]
    labelCategory = createLabelWithCategory(builder, category, labels)
    dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
    instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

    # 4. Create a query with parameters
    # all parameters are optional
    # with all parameters set (especially with the data and instance uuids set) the result of the query will be empty. Set the query parameters to adequate values or remove them from the query creation
    query = createQuery(
        builder,
        # boundingBox=boundingboxStamped,
        # timeInterval=timeInterval,
        # labels=labelCategory,
        # mustHaveAllLabels=True,
        projectUuids=projectUuids,
        # instanceUuids=instanceUuids,
        # dataUuids=dataUuids,
        withoutData=True,
        fullyEncapsulated=False
    )
    builder.Finish(query)
    buf = builder.Output()

    # 5. Query the server for images matching the query and iterate over them
    for responseBuf in stub.GetImage(bytes(buf)):
        response = Image.Image.GetRootAs(responseBuf)

        print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
        print(response.LabelsBbLength())
        if response.LabelsBbLength() > 0:
            print(
                "first label: "
                + response.LabelsBb(0).BoundingBox2dLabeled(0).LabelWithInstance().Label().Label().decode("utf-8")
                + " ; confidence: "
                + str(response.LabelsBb(0).BoundingBox2dLabeled(0).LabelWithInstance().Label().Confidence())
            )
            print(
                "first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): "
                + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().CenterPoint().X())
                + " "
                + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().CenterPoint().Y())
                + " "
                + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().SpatialExtent().X())
                + " "
                + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().SpatialExtent().Y())
                + " "
                + str(response.LabelsBb(0).BoundingBox2dLabeled(0).BoundingBox().Rotation())
                + "\n"
            )

    print("done.")
    ```

    Output:

    ```shell
    testproject 3af70ba8-1e81-4f60-86d2-a4257d88f01e

    uuidmsg: 00ced216-40b1-4d54-817f-11c413b228c6
    first label: testlabel0
    first bounding box (Xmin,Ymin,Xmax,Ymax): 0.01 0.02 0.03 0.04

    uuidmsg: 5836f989-adbb-46a0-a689-9e0527d457fe
    first label: testlabel0
    first bounding box (Xmin,Ymin,Xmax,Ymax): 0.01 0.02 0.03 0.04

    uuidmsg: 5b39487d-238a-43e4-a8a4-0f7efb876e8b
    first label: testlabel0
    first bounding box (Xmin,Ymin,Xmax,Ymax): 0.01 0.02 0.03 0.04

    uuidmsg: 5d330208-e534-4bfb-b242-00295e6b027d
    first label: testlabel0
    first bounding box (Xmin,Ymin,Xmax,Ymax): 0.01 0.02 0.03 0.04

    done.
    ```
