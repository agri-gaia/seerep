# Sending & querying images

## Sending images

In this example we will send images with labeled bounding boxes as well as
general labels to SEEREP.

In order to save images, we need to mandatorily provide the intrinsics of the
camera used to capture them. After the successfully saving the camera intrinsics,
we need to provide the uuid of it along with the images. SEEREP will ensure that
the Camera Intrinsics UUID provided with an image has a UUID stored against it.

Additionally we add some coordinate transformations at the end.

Source:
[examples/python/gRPC/images/gRPC_pb_sendLabeledImage.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/images/gRPC_pb_sendLabeledImage.py)

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_pb_sendLabeledImage.py"
```

Output:

```txt
testproject 842de425-2d50-4adf-8aa3-6df257a7c76c
testproject ff739be8-cf0c-4657-bff0-f66f3e7f578d

camera intrinsics will be saved against the uuid(s): {'909c8439-36b6-44ff-9561-f7921de5d8e8'}
the uuid of the sent image number 0 is: 261a7b2c-297b-42db-93da-770e5f7f53cf
the uuid of the sent image number 1 is: feddb62d-fdfa-494a-8e4a-e1b2209134a6
the uuid of the sent image number 2 is: 6956ea01-b834-461f-85cf-5621e04f02fb
the uuid of the sent image number 3 is: 111e62dd-9aba-4fde-ae3d-ed3e3b6041f8
the uuid of the sent image number 4 is: 67138174-08a9-4943-848a-736bddd755b9
the uuid of the sent image number 5 is: 5e139007-9eea-4e71-9c5a-44f08c93027e
the uuid of the sent image number 6 is: c10eff49-4412-425d-99ef-d8b3f95c3806
the uuid of the sent image number 7 is: 708be3d8-7ddb-4ef5-9b00-37021b7b83ff
the uuid of the sent image number 8 is: 4ce47c5c-60d2-4f78-810f-ac730136f462
the uuid of the sent image number 9 is: 6c9ca46b-84fb-40d2-8523-2ad07284a43c
```

## Query images

Now we will query the previously send images with some criteria. Possible query
parameters are:

### 2D Polygon (spatial query)

Spatial queries in SEEREP are performed using a 2D polygon. This polygon should
be simple (no more than 2 vertices on the same edge) and convex (no edges curving
inward). This 2D polygon lies on a interval on the z-axis defined through a
z-coordinate point and a height value. Queries are performed by forming an
encompassing axis aligned bounding box from the polygon. This can lead to an
AABB larger than the polygon and poses the potential problem of returning results
to the user which are not fully inside the query polygon. That problem is
resolved by providing a boolean variable called fullyEncapsulated`. If false,
resultant polygons, which are partially inside the query polygon are also returned.

### A time interval (temporal query)

Temporal queries in SEEREP are performed using a time interval. When using image
queries the stamp in the `header` of those images is used and when that time
lies in the interval the image is returned as part of the response. The interval
is closed.

### Labels (semantic query)

### ProjectUuids

Only performs the query in these included projects specified by their uuids.

### WithoutData

If the pixel data of the image should not be returned in order to save bandwith.

### InMapFrame

Whether the query is done in the map frame.
If not, the provided polygon for the spatial query will be transformed from the
geodesic coordinates of the project into the map frame beforehand.

### Example code for querying images

<!-- markdownlint-disable -->

=== "Flatbuffers"

    Source: [examples/python/gRPC/images/gRPC_fb_queryImage.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/images/gRPC_fb_queryImage.py)

    ```python
    --8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_fb_queryImage.py"
    ```

    After sending the images, executing the query script results in the following output:

    ```txt
    count of images: 10
    ------------------------------------------------------------------
    uuidmsg: 111e62dd-9aba-4fde-ae3d-ed3e3b6041f8
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: 261a7b2c-297b-42db-93da-770e5f7f53cf
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: 4ce47c5c-60d2-4f78-810f-ac730136f462
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: 5e139007-9eea-4e71-9c5a-44f08c93027e
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: 67138174-08a9-4943-848a-736bddd755b9
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: 6956ea01-b834-461f-85cf-5621e04f02fb
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: 6c9ca46b-84fb-40d2-8523-2ad07284a43c
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: 708be3d8-7ddb-4ef5-9b00-37021b7b83ff
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: c10eff49-4412-425d-99ef-d8b3f95c3806
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    uuidmsg: feddb62d-fdfa-494a-8e4a-e1b2209134a6
    count of labels: 2
    first label: label1
    ------------------------------------------------------------------
    ```

=== "Protocol Buffers"

    Source:
    [examples/python/gRPC/images/gRPC_pb_queryImage.py](https://github.com/agri-gaia/seerep/blob/main/examples/python/gRPC/images/gRPC_pb_queryImage.py)

    ```python
    --8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/examples/python/gRPC/images/gRPC_pb_queryImage.py"
    ```

    After sending the images, executing the query script results in the
    following output:

    ```txt
    count of images 8
    uuidmsg: 111e62dd-9aba-4fde-ae3d-ed3e3b6041f8
    first label: label1
    uuidmsg: 261a7b2c-297b-42db-93da-770e5f7f53cf
    first label: label1
    uuidmsg: 5e139007-9eea-4e71-9c5a-44f08c93027e
    first label: label1
    uuidmsg: 67138174-08a9-4943-848a-736bddd755b9
    first label: label1
    uuidmsg: 6956ea01-b834-461f-85cf-5621e04f02fb
    first label: label1
    uuidmsg: 708be3d8-7ddb-4ef5-9b00-37021b7b83ff
    first label: label1
    uuidmsg: c10eff49-4412-425d-99ef-d8b3f95c3806
    first label: label1
    uuidmsg: feddb62d-fdfa-494a-8e4a-e1b2209134a6
    first label: label1
    ```
