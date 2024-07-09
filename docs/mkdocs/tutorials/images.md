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
testproject 28600b43-6385-4693-b617-c6209d9bde87
testproject 2292aa91-c369-4cb2-9de8-6987218fe268

camera intrinsics will be saved against the uuid(s): {'085a41c6-b81c-4a8e-a16c-fb71d99b0ffe'}
the uuid of the sent image number 0 is: 6821ea73-c9ca-45b3-97da-727ebe46cde5
the uuid of the sent image number 1 is: 1d5d2714-f9d3-45a5-b0fd-a45483057bd1
the uuid of the sent image number 2 is: 11251495-64d2-48be-8b3b-7ad06d360f02
the uuid of the sent image number 3 is: 2a2d8b62-3eb5-4bd8-aeeb-e8a4eab64322
the uuid of the sent image number 4 is: baadd923-b2ea-484c-9794-58788ce79980
the uuid of the sent image number 5 is: bdc82995-d827-41b3-9fd8-433e79dbced6
the uuid of the sent image number 6 is: 7678649c-39c9-4553-952e-cd93ed8ed675
the uuid of the sent image number 7 is: 1e5ff01d-ef2a-44e1-9af8-99496d9ecc2a
the uuid of the sent image number 8 is: c42b98ad-d92c-4b0c-9ea5-d6e6d4cff6e8
the uuid of the sent image number 9 is: b9619d15-5421-47db-a08b-ffb9f5cc8f3e
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
    count of images: 6
    --------------------------------------------------------------------------------------------
    uuidmsg: 680128d1-5a86-457e-bd7f-be1d81371a30
    count of bounding box labels: 2
    first label: testlabel0 ; confidence: 0.0
    first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): 0.01 0.02 0.03 0.04 0.0
    --------------------------------------------------------------------------------------------
    uuidmsg: 6d6a9c09-3781-4e3e-a5df-948ab821b149
    count of bounding box labels: 2
    first label: testlabel0 ; confidence: 0.0
    first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): 0.01 0.02 0.03 0.04 0.0
    --------------------------------------------------------------------------------------------
    uuidmsg: 914273f7-96d3-453e-9a30-b4b411a81afe
    count of bounding box labels: 2
    first label: testlabel0 ; confidence: 0.0
    first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): 0.01 0.02 0.03 0.04 0.0
    --------------------------------------------------------------------------------------------
    uuidmsg: ad0db9ee-d0d8-46c2-9c13-518e29d3cc2b
    count of bounding box labels: 2
    first label: testlabel0_ ; confidence: 0.0
    first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): 0.01 0.02 0.03 0.04 0.0
    --------------------------------------------------------------------------------------------
    uuidmsg: b371b172-08d2-47c7-a750-db77149af897
    count of bounding box labels: 2
    first label: testlabel0 ; confidence: 0.0
    first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): 0.01 0.02 0.03 0.04 0.0
    --------------------------------------------------------------------------------------------
    uuidmsg: f1e755b2-fa6b-4cf0-a0f5-fc928be84850
    count of bounding box labels: 2
    first label: testlabel0 ; confidence: 0.0
    first bounding box (Xcenter,Ycenter,Xextent,Yextent, rotation): 0.01 0.02 0.03 0.04 0.0
    --------------------------------------------------------------------------------------------
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
    uuidmsg: 680128d1-5a86-457e-bd7f-be1d81371a30
    first label: testlabel0
    first label confidence: 0.0
    First bounding box (Xcenter,Ycenter,Xextent,Yextent): 0.01 0.02 0.03 0.04

    uuidmsg: 6d6a9c09-3781-4e3e-a5df-948ab821b149
    first label: testlabel0
    first label confidence: 0.0
    First bounding box (Xcenter,Ycenter,Xextent,Yextent): 0.01 0.02 0.03 0.04

    uuidmsg: 914273f7-96d3-453e-9a30-b4b411a81afe
    first label: testlabel0
    first label confidence: 0.0
    First bounding box (Xcenter,Ycenter,Xextent,Yextent): 0.01 0.02 0.03 0.04

    uuidmsg: ad0db9ee-d0d8-46c2-9c13-518e29d3cc2b
    first label: testlabel0_
    first label confidence: 0.0
    First bounding box (Xcenter,Ycenter,Xextent,Yextent): 0.01 0.02 0.03 0.04

    uuidmsg: b371b172-08d2-47c7-a750-db77149af897
    first label: testlabel0
    first label confidence: 0.0
    First bounding box (Xcenter,Ycenter,Xextent,Yextent): 0.01 0.02 0.03 0.04

    uuidmsg: b6abe511-ba53-4782-82d3-53fb80887754
    first label: testlabel0
    first label confidence: 0.0
    First bounding box (Xcenter,Ycenter,Xextent,Yextent): 0.01 0.02 0.03 0.04

    uuidmsg: cf1d977f-5356-4edc-a02b-e4b454547f63
    first label: testlabel0_
    first label confidence: 0.0
    First bounding box (Xcenter,Ycenter,Xextent,Yextent): 0.01 0.02 0.03 0.04

    uuidmsg: f1e755b2-fa6b-4cf0-a0f5-fc928be84850
    first label: testlabel0
    first label confidence: 0.0
    First bounding box (Xcenter,Ycenter,Xextent,Yextent): 0.01 0.02 0.03 0.04
    ```
