# Protocol Documentation
<a name="top"></a>

## Table of Contents

- [Header.proto](#Header.proto)
    - [Header](#ag.Header)
  
- [Image.proto](#Image.proto)
    - [Image](#ag.Image)
  
- [Point.proto](#Point.proto)
    - [Point](#ag.Point)
  
- [PointCloud2.proto](#PointCloud2.proto)
    - [PointCloud2](#ag.PointCloud2)
  
- [PointField.proto](#PointField.proto)
    - [PointField](#ag.PointField)
  
    - [PointField.Datatype](#ag.PointField.Datatype)
  
- [PointStamped.proto](#PointStamped.proto)
    - [PointStamped](#ag.PointStamped)
  
- [Polygon.proto](#Polygon.proto)
    - [Polygon](#ag.Polygon)
  
- [PolygonStamped.proto](#PolygonStamped.proto)
    - [PolygonStamped](#ag.PolygonStamped)
  
- [Pose.proto](#Pose.proto)
    - [Pose](#ag.Pose)
  
- [PoseStamped.proto](#PoseStamped.proto)
    - [PoseStamped](#ag.PoseStamped)
  
- [Quaternion.proto](#Quaternion.proto)
    - [Quaternion](#ag.Quaternion)
  
- [QuaternionStamped.proto](#QuaternionStamped.proto)
    - [QuaternionStamped](#ag.QuaternionStamped)
  
- [Transform.proto](#Transform.proto)
    - [Transform](#ag.Transform)
  
- [TransformStamped.proto](#TransformStamped.proto)
    - [TransformStamped](#ag.TransformStamped)
  
- [Twist.proto](#Twist.proto)
    - [Twist](#ag.Twist)
  
- [TwistStamped.proto](#TwistStamped.proto)
    - [TwistStamped](#ag.TwistStamped)
  
- [TwistWithCovariance.proto](#TwistWithCovariance.proto)
    - [TwistWithCovariance](#ag.TwistWithCovariance)
  
- [TwistWithCovarianceStamped.proto](#TwistWithCovarianceStamped.proto)
    - [TwistWithCovarianceStamped](#ag.TwistWithCovarianceStamped)
  
- [Vector3.proto](#Vector3.proto)
    - [Vector3](#ag.Vector3)
  
- [Vector3Stamped.proto](#Vector3Stamped.proto)
    - [Vector3Stamped](#ag.Vector3Stamped)
  
- [Scalar Value Types](#scalar-value-types)



<a name="Header.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Header.proto



<a name="ag.Header"></a>

### Header
Standard metadata for higher-level stamped data types.
This is generally used to communicate timestamped data
in a particular coordinate frame.


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| seq | [uint32](#uint32) |  | sequence id, consecutively increasing id |
| stamp | [google.protobuf.Timestamp](#google.protobuf.Timestamp) |  | time this data is associated with |
| frame_id | [string](#string) |  | name of the coordinate frame this data is associated with |





 

 

 

 



<a name="Image.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Image.proto



<a name="ag.Image"></a>

### Image



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  | Header timestamp should be acquisition time of image Header frame_id should be optical frame of camera origin of frame should be optical center of camera &#43;x should point to the right in the image &#43;y should point down in the image &#43;z should point into to plane of the image If the frame_id here and the frame_id of the CameraInfo message associated with the image conflict the behavior is undefined |
| height | [uint32](#uint32) |  | image height, that is, number of rows |
| width | [uint32](#uint32) |  | image width, that is, number of columns |
| encoding | [string](#string) |  | Encoding of pixels -- channel meaning, ordering, size, e.g. rgb8, rgba8, bgr8, bgra8, mono8, mono16 see http://docs.ros.org/en/diamondback/api/sensor_msgs/html/image__encodings_8cpp_source.html |
| is_bigendian | [bool](#bool) |  | is this data bigendian? |
| step | [uint32](#uint32) |  | Full row length in bytes |
| row_step | [uint32](#uint32) |  |  |
| data | [bytes](#bytes) |  | actual matrix data, size is (step * rows) |





 

 

 

 



<a name="Point.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Point.proto



<a name="ag.Point"></a>

### Point



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [double](#double) |  |  |
| y | [double](#double) |  |  |
| z | [double](#double) |  |  |





 

 

 

 



<a name="PointCloud2.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## PointCloud2.proto



<a name="ag.PointCloud2"></a>

### PointCloud2



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| height | [uint32](#uint32) |  |  |
| width | [uint32](#uint32) |  |  |
| fields | [PointField](#ag.PointField) | repeated |  |
| is_bigendian | [bool](#bool) |  |  |
| point_step | [uint32](#uint32) |  |  |
| row_step | [uint32](#uint32) |  |  |
| data | [bytes](#bytes) |  | the point cloud data |
| is_dense | [bool](#bool) |  |  |





 

 

 

 



<a name="PointField.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## PointField.proto



<a name="ag.PointField"></a>

### PointField



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| name | [string](#string) |  |  |
| offset | [uint32](#uint32) |  |  |
| datatype | [PointField.Datatype](#ag.PointField.Datatype) |  |  |
| count | [uint32](#uint32) |  |  |





 


<a name="ag.PointField.Datatype"></a>

### PointField.Datatype


| Name | Number | Description |
| ---- | ------ | ----------- |
| UNSET | 0 |  |
| INT8 | 1 |  |
| UINT8 | 2 |  |
| INT16 | 3 |  |
| UINT16 | 4 |  |
| INT32 | 5 |  |
| UINT32 | 6 |  |
| FLOAT32 | 7 |  |
| FLOAT64 | 8 |  |


 

 

 



<a name="PointStamped.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## PointStamped.proto



<a name="ag.PointStamped"></a>

### PointStamped



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| point | [Point](#ag.Point) |  |  |





 

 

 

 



<a name="Polygon.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Polygon.proto



<a name="ag.Polygon"></a>

### Polygon



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| points | [Point](#ag.Point) | repeated |  |





 

 

 

 



<a name="PolygonStamped.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## PolygonStamped.proto



<a name="ag.PolygonStamped"></a>

### PolygonStamped



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| polygon | [Polygon](#ag.Polygon) |  |  |





 

 

 

 



<a name="Pose.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Pose.proto



<a name="ag.Pose"></a>

### Pose



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| position | [Point](#ag.Point) |  |  |
| orientation | [Quaternion](#ag.Quaternion) |  |  |





 

 

 

 



<a name="PoseStamped.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## PoseStamped.proto



<a name="ag.PoseStamped"></a>

### PoseStamped



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| pose | [Pose](#ag.Pose) |  |  |





 

 

 

 



<a name="Quaternion.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Quaternion.proto



<a name="ag.Quaternion"></a>

### Quaternion



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [double](#double) |  |  |
| y | [double](#double) |  |  |
| z | [double](#double) |  |  |
| w | [double](#double) |  |  |





 

 

 

 



<a name="QuaternionStamped.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## QuaternionStamped.proto



<a name="ag.QuaternionStamped"></a>

### QuaternionStamped



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| quaternion | [Quaternion](#ag.Quaternion) |  |  |





 

 

 

 



<a name="Transform.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Transform.proto



<a name="ag.Transform"></a>

### Transform



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| translation | [Vector3](#ag.Vector3) |  |  |
| rotation | [Quaternion](#ag.Quaternion) |  |  |





 

 

 

 



<a name="TransformStamped.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## TransformStamped.proto



<a name="ag.TransformStamped"></a>

### TransformStamped



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| child_frame_id | [string](#string) |  |  |
| transform | [Transform](#ag.Transform) |  |  |





 

 

 

 



<a name="Twist.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Twist.proto



<a name="ag.Twist"></a>

### Twist



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| linear | [Vector3](#ag.Vector3) |  |  |
| angular | [Vector3](#ag.Vector3) |  |  |





 

 

 

 



<a name="TwistStamped.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## TwistStamped.proto



<a name="ag.TwistStamped"></a>

### TwistStamped



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| twist | [Twist](#ag.Twist) |  |  |





 

 

 

 



<a name="TwistWithCovariance.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## TwistWithCovariance.proto



<a name="ag.TwistWithCovariance"></a>

### TwistWithCovariance
This expresses velocity in free space with uncertainty.


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| twist | [Twist](#ag.Twist) |  |  |
| covariance | [double](#double) | repeated | Row-major representation of the 6x6 covariance matrix The orientation parameters use a fixed-axis representation. In order, the parameters are: x, y, z, roll (x), pitch (y), yaw (z) |





 

 

 

 



<a name="TwistWithCovarianceStamped.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## TwistWithCovarianceStamped.proto



<a name="ag.TwistWithCovarianceStamped"></a>

### TwistWithCovarianceStamped



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| twist | [TwistWithCovariance](#ag.TwistWithCovariance) |  |  |





 

 

 

 



<a name="Vector3.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Vector3.proto



<a name="ag.Vector3"></a>

### Vector3



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [double](#double) |  |  |
| y | [double](#double) |  |  |
| z | [double](#double) |  |  |





 

 

 

 



<a name="Vector3Stamped.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## Vector3Stamped.proto



<a name="ag.Vector3Stamped"></a>

### Vector3Stamped



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| header | [Header](#ag.Header) |  |  |
| vector | [Vector3](#ag.Vector3) |  |  |





 

 

 

 



## Scalar Value Types

| .proto Type | Notes | C++ | Java | Python | Go | C# | PHP | Ruby |
| ----------- | ----- | --- | ---- | ------ | -- | -- | --- | ---- |
| <a name="double" /> double |  | double | double | float | float64 | double | float | Float |
| <a name="float" /> float |  | float | float | float | float32 | float | float | Float |
| <a name="int32" /> int32 | Uses variable-length encoding. Inefficient for encoding negative numbers – if your field is likely to have negative values, use sint32 instead. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="int64" /> int64 | Uses variable-length encoding. Inefficient for encoding negative numbers – if your field is likely to have negative values, use sint64 instead. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="uint32" /> uint32 | Uses variable-length encoding. | uint32 | int | int/long | uint32 | uint | integer | Bignum or Fixnum (as required) |
| <a name="uint64" /> uint64 | Uses variable-length encoding. | uint64 | long | int/long | uint64 | ulong | integer/string | Bignum or Fixnum (as required) |
| <a name="sint32" /> sint32 | Uses variable-length encoding. Signed int value. These more efficiently encode negative numbers than regular int32s. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="sint64" /> sint64 | Uses variable-length encoding. Signed int value. These more efficiently encode negative numbers than regular int64s. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="fixed32" /> fixed32 | Always four bytes. More efficient than uint32 if values are often greater than 2^28. | uint32 | int | int | uint32 | uint | integer | Bignum or Fixnum (as required) |
| <a name="fixed64" /> fixed64 | Always eight bytes. More efficient than uint64 if values are often greater than 2^56. | uint64 | long | int/long | uint64 | ulong | integer/string | Bignum |
| <a name="sfixed32" /> sfixed32 | Always four bytes. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="sfixed64" /> sfixed64 | Always eight bytes. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="bool" /> bool |  | bool | boolean | boolean | bool | bool | boolean | TrueClass/FalseClass |
| <a name="string" /> string | A string must always contain UTF-8 encoded or 7-bit ASCII text. | string | String | str/unicode | string | string | string | String (UTF-8) |
| <a name="bytes" /> bytes | May contain any arbitrary sequence of bytes. | string | ByteString | str | []byte | ByteString | string | String (ASCII-8BIT) |

