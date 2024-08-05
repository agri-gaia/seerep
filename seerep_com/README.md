<!-- markdownlint-disable-file MD024 -->
# gRPC API

This page provides a summary of the available gRPC services offered by SEEREP.
It is **recommended to use FlatBuffers for serialization**, as the latest
updates and features utilize it.
Protocol Buffers might lack certain features and could potentially be
discontinued in the future, as indicated by
[issue#372](https://github.com/agri-gaia/seerep/issues/372).
The services are implemented in the [seerep_server](https://github.com/agri-gaia/seerep/tree/main/seerep_srv/seerep_server/src)
package.

The *streaming* column in the following tables indicates whether the client or
server can send `1 ... N` messages during the service call.

## Flatbuffers

### MetaOperations

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [CreateProject](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/meta_operations.fbs)    | Create a new SEEREP project    | [ProjectCreation](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/projectCreation.fbs)     | [ProjetInfo](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/project_info.fbs) | - |
| [GetProjects](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/meta_operations.fbs)    | Get all current projects    | [Empty](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/empty.fbs)     | [ProjetInfos](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/project_infos.fbs) | - |
| [LoadProjects](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/meta_operations.fbs)    | Load all unindexed projects    | [Empty](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/empty.fbs)     | [ProjetInfos](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/project_infos.fbs) | - |
| [DeleteProject](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/meta_operations.fbs)    | Delete a project   | [ProjectInfo](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/project_info.fbs)     | [Empty](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/empty.fbs) | - |
| [GetOverallTimeInterval](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/meta_operations.fbs)    | Get the total timespan of a data type   | [UuidDatatypePair](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/uuid_datatype_pair.fbs)     | [TimeInterval](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/time_interval.fbs) | - |
| [GetOverallBoundingBox](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/meta_operations.fbs)    | Get the full extent of a data type  | [UuidDatatypePair](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/uuid_datatype_pair.fbs)     | [Boundingbox](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/boundingbox.fbs) | - |
| [GetAllCategories](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/meta_operations.fbs)    | Get all categories of a data type   | [UuidDatatypePair](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/uuid_datatype_pair.fbs)     | [StringVector](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/string_vector.fbs) | - |
| [GetAllLabels](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/meta_operations.fbs)    | Get all labels in a category of a data type   | [UuidDatatypeWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/uuid_datatype_with_category.fbs)     | [StringVector](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/string_vector.fbs) | - |

### ImageService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [GetImage](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/image_service.fbs)    | Get images based on a query   | [Query](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/query.fbs)     | [Image](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/image.fbs) | Server |
| [TransferImage](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/image_service.fbs)    | Transfer images to SEEREP   | [Image](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/image.fbs)     | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/server_response.fbs) | Client |
| [AddLabels](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/image_service.fbs)    | Add labels to existing images| [DatasetUuidLabel](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/dataset_uuid_label.fbs)     | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/server_response.fbs) | Client |

### CameraIntrinsicsService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [GetCameraIntrinsics](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/camera_intrinsics_service.fbs)    | Get a specific camera instrinic    | [CameraIntrinsicsQuery](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/camera_intrinsics_query.fbs) | [CameraIntrinsics](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/camera_intrinsics.fbs) | - |
| [TransferCameraIntrinsics](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/camera_intrinsics_service.fbs)    | Transfer camera intrinsics to SEEREP   |  [CameraIntrinsics](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/camera_intrinsics.fbs) | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/server_response.fbs) | - |

### PointCloudService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [GetPointCloud2](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/point_cloud_service.fbs)    | Get point clouds based on a query   | [Query](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/query.fbs)     | [PointCloud2](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_cloud_2.fbs) | Server |
| [TransferPointCloud2](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/point_cloud_service.fbs)    | Transfer point clouds to SEEREP   | [PointCloud2](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_cloud_2.fbs)     | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/server_response.fbs) | Client |
| [AddLabels](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/point_cloud_service.fbs)    | Add labels to existing point clouds | [DatasetUuidLabel](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/dataset_uuid_label.fbs)     | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/server_response.fbs) | Client |

### TfService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [TransferTransformStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/tf_service.fbs)    | Add a transformation to SEEREP  | [TransformStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/transform_stamped.fbs)     | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/server_response.fbs) | Client |
| [GetFrames](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/tf_service.fbs)    | Get all frames of a project   | [FrameQuery](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/frame_query.fbs)     |  [StringVector](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/string_vector.fbs) | - |
| [GetTransformStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/tf_service.fbs)    | Get a transformation from SEEREP | [TransformStampedQuery](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/transform_stamped_query.fbs)     | [TransformStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/transform_stamped.fbs) | - |

### PointService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [GetPoint](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/point_service.fbs)    | Get points based on a query  | [Query](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/query.fbs)      | [PointStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_stamped.fbs) | Server |
| [TransferPoint](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/point_service.fbs)    | Transfer points to SEEREP   | [PointStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/point_stamped.fbs)   | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/server_response.fbs)  | Client |
| [AddAttribute](https://github.com/agri-gaia/seerep/blob/main/seerep_com/fbs/point_service.fbs)    | Add attribute to existing points | [AttributesStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/attributes_stamped.fbs) | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/server_response.fbs) | Client |

## Protocol Buffers

### MetaOperations

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [CreateProject](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/meta_operations.proto)    | Create a new SEEREP project    | [ProjectCreation](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/projectCreation.proto)     | [ProjetInfo](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/project_info.proto) | - |
| [GetProjects](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/meta_operations.proto)    | Get all current projects    | [Empty](https://protobuf.dev/reference/protobuf/google.protobuf/#empty)     | [ProjetInfos](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/project_infos.proto) | - |
| [GetOverallTimeInterval](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/meta_operations.proto)    | Get the total timespan of a data type   | [UuidDatatypePair](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/uuid_datatype_pair.proto)     | [TimeInterval](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/time_interval.proto) | - |
| [GetOverallBoundingBox](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/meta_operations.proto)    | Get the full extent of a data type  | [UuidDatatypePair](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/uuid_datatype_pair.proto)     | [Boundingbox](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/boundingbox.proto) | - |
| [GetAllCategories](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/meta_operations.proto)    | Get all categories of a data type   | [UuidDatatypePair](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/uuid_datatype_pair.proto)     | [StringVector](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/string_vector.proto) | - |
| [GetAllLabels](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/meta_operations.proto)    | Get all labels in a category of a data type   | [UuidDatatypeWithCategory](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/uuid_datatype_with_category.proto)     | [StringVector](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/string_vector.proto) | - |

### ImageService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [GetImage](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/image_service.proto)    | Get images based on a query   | [Query](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/query.proto)     | [Image](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/image.proto) | Server |
| [TransferImage](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/image_service.proto)    | Transfer images to SEEREP   | [Image](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/image.proto)     | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/server_response.proto) | - |

### CameraIntrinsicsService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [GetCameraIntrinsics](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/camera_intrinsics_service.proto)    | Get a specific camera instrinic    | [CameraIntrinsicsQuery](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/camera_intrinsics_query.proto) | [CameraIntrinsics](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/camera_intrinsics.proto) | - |
| [TransferCameraIntrinsics](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/camera_intrinsics_service.proto)    | Transfer camera intrinsics to SEEREP   |  [CameraIntrinsics](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/camera_intrinsics.proto) | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/server_response.proto) | - |

### PointCloudService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [GetPointCloud2](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/point_cloud_service.proto)    | Get point clouds based on a query   | [Query](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/query.proto)     | [PointCloud2](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/point_cloud_2.proto) | Server |
| [TransferPointCloud2](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/point_cloud_service.proto)    | Transfer point clouds to SEEREP   | [PointCloud2](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/point_cloud_2.proto)     | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/server_response.proto) | - |

### TfService

| Service                | Description              | Request Message                                                                                          | Response Message | Streaming |
|------------------------|--------------------------------|----------------------------------------------------------------------------------------------------------|------------------|-----------|
| [TransferTransformStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/tf_service.proto)    | Add a transformation to SEEREP  | [TransformStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/transform_stamped.proto)     | [ServerResponse](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/server_response.proto) | - |
| [GetFrames](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/tf_service.proto)    | Get all frames of a project   | [FrameQuery](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/frame_query.proto)     |  [StringVector](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/string_vector.proto) | - |
| [GetTransformStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_com/protos/tf_service.proto)    | Get a transformation from SEEREP | [TransformStampedQuery](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/transform_stamped_query.proto)     | [TransformStamped](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/protos/transform_stamped.proto) | - |
