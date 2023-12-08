#include "seerep_ros_conversions_fb/conversions.h"

namespace seerep_ros_conversions_fb
{
/*
 * Header
 */
flatbuffers::grpc::Message<seerep::fb::Header> toFlat(const std_msgs::Header& header, std::string projectuuid,
                                                      std::string msguuid = "")
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(header, projectuuid, builder, msguuid));
  return builder.ReleaseMessage<seerep::fb::Header>();
}
flatbuffers::Offset<seerep::fb::Header> toFlat(const std_msgs::Header& header, std::string projectuuid,
                                               flatbuffers::grpc::MessageBuilder& builder, std::string msguuid = "")
{
  seerep::fb::TimestampBuilder timestampbuilder(builder);
  timestampbuilder.add_seconds(header.stamp.sec);
  timestampbuilder.add_nanos(header.stamp.nsec);
  auto stamp = timestampbuilder.Finish();

  auto frameIdOffset = builder.CreateString(header.frame_id);
  auto projectUuidOffset = builder.CreateString(projectuuid);
  auto msgUuidOffset = builder.CreateString(msguuid);

  seerep::fb::HeaderBuilder headerbuilder(builder);
  headerbuilder.add_seq(header.seq);
  headerbuilder.add_frame_id(frameIdOffset);
  headerbuilder.add_stamp(stamp);
  headerbuilder.add_uuid_project(projectUuidOffset);
  headerbuilder.add_uuid_msgs(msgUuidOffset);

  return headerbuilder.Finish();
}

std_msgs::Header toROS(const seerep::fb::Header& header)
{
  std_msgs::Header ret;
  ret.seq = header.seq();
  ret.frame_id = header.frame_id()->str();
  ret.stamp.sec = header.stamp()->seconds();
  ret.stamp.nsec = header.stamp()->nanos();
  return ret;
}

/*
 * PointField
 */
flatbuffers::grpc::Message<seerep::fb::PointField> toFlat(const sensor_msgs::PointField& point_field)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(point_field, builder));
  return builder.ReleaseMessage<seerep::fb::PointField>();
}
flatbuffers::Offset<seerep::fb::PointField> toFlat(const sensor_msgs::PointField& point_field,
                                                   flatbuffers::grpc::MessageBuilder& builder)

{
  auto nameOffset = builder.CreateString(point_field.name);

  seerep::fb::PointFieldBuilder pointFieldBuilder(builder);
  pointFieldBuilder.add_name(nameOffset);
  pointFieldBuilder.add_offset(point_field.offset);
  pointFieldBuilder.add_datatype(seerep::fb::Point_Field_Datatype(point_field.datatype));
  pointFieldBuilder.add_count(point_field.count);
  return pointFieldBuilder.Finish();
}

sensor_msgs::PointField toROS(const seerep::fb::PointField& point_field)
{
  sensor_msgs::PointField ret;
  ret.name = point_field.name()->str();
  ret.offset = point_field.offset();
  ret.datatype = point_field.datatype();
  ret.count = point_field.count();
  return ret;
}

/*
 * PointCloud2
 */
flatbuffers::grpc::Message<seerep::fb::PointCloud2> toFlat(const sensor_msgs::PointCloud2& cloud,
                                                           std::string projectuuid, const std::string& msguuid)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(cloud, projectuuid, builder, msguuid));
  return builder.ReleaseMessage<seerep::fb::PointCloud2>();
}
flatbuffers::Offset<seerep::fb::PointCloud2> toFlat(const sensor_msgs::PointCloud2& cloud, std::string projectuuid,
                                                    flatbuffers::grpc::MessageBuilder& builder,
                                                    const std::string& msguuid)
{
  auto header = toFlat(cloud.header, projectuuid, builder, msguuid);

  std::vector<flatbuffers::Offset<seerep::fb::PointField>> fieldsvector;
  fieldsvector.reserve(cloud.fields.size());
  for (auto field : cloud.fields)
  {
    fieldsvector.push_back(toFlat(field, builder));
  }
  auto fieldsOffset = builder.CreateVector(fieldsvector);
  auto dataOffset = builder.CreateVector(cloud.data);

  seerep::fb::PointCloud2Builder cloudbuilder(builder);
  cloudbuilder.add_header(header);
  cloudbuilder.add_height(cloud.height);
  cloudbuilder.add_width(cloud.width);
  cloudbuilder.add_is_bigendian(cloud.is_bigendian);
  cloudbuilder.add_point_step(cloud.point_step);
  cloudbuilder.add_row_step(cloud.row_step);
  cloudbuilder.add_is_dense(cloud.is_dense);
  cloudbuilder.add_fields(fieldsOffset);
  cloudbuilder.add_data(dataOffset);

  return cloudbuilder.Finish();
}

sensor_msgs::PointCloud2 toROS(const seerep::fb::PointCloud2& cloud)
{
  sensor_msgs::PointCloud2 ret;
  ret.header = toROS(*cloud.header());
  ret.height = cloud.height();
  ret.width = cloud.width();
  for (auto field : *cloud.fields())
  {
    ret.fields.push_back(toROS(*field));
  }
  ret.is_bigendian = cloud.is_bigendian();
  ret.point_step = cloud.point_step();
  ret.row_step = cloud.row_step();
  // std::copy(cloud.data().begin(), cloud.data().end(), std::back_inserter(ret.data));
  std::copy_n(cloud.data()->Data(), cloud.data()->size(), std::back_inserter(ret.data));
  ret.is_dense = cloud.is_dense();
  return ret;
}

/*
 * Image
 */
flatbuffers::grpc::Message<seerep::fb::Image> toFlat(const sensor_msgs::Image& image, std::string projectuuid,
                                                     std::string cameraInstrinsicUuid, std::string msguuid)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(image, projectuuid, builder, cameraInstrinsicUuid, msguuid));
  return builder.ReleaseMessage<seerep::fb::Image>();
}
flatbuffers::Offset<seerep::fb::Image> toFlat(const sensor_msgs::Image& image, std::string projectuuid,
                                              flatbuffers::grpc::MessageBuilder& builder,
                                              std::string cameraInstrinsicUuid, std::string msguuid)
{
  auto header = toFlat(image.header, projectuuid, builder, msguuid);

  auto encodingOffset = builder.CreateString(image.encoding);
  auto dataVector = builder.CreateVector(image.data);

  auto cameraIntrinsicOffset = builder.CreateString(cameraInstrinsicUuid);

  seerep::fb::ImageBuilder imagebuilder(builder);
  imagebuilder.add_header(header);
  imagebuilder.add_height(image.height);
  imagebuilder.add_width(image.width);
  imagebuilder.add_encoding(encodingOffset);
  imagebuilder.add_is_bigendian(image.is_bigendian);
  imagebuilder.add_step(image.step);
  imagebuilder.add_data(dataVector);
  imagebuilder.add_uuid_cameraintrinsics(cameraIntrinsicOffset);

  return imagebuilder.Finish();
}

sensor_msgs::Image toROS(const seerep::fb::Image& image)
{
  sensor_msgs::Image ret;
  ret.header = toROS(*image.header());
  ret.height = image.height();
  ret.width = image.width();
  ret.encoding = image.encoding()->str();
  ret.is_bigendian = image.is_bigendian();
  ret.step = image.step();
  ret.data.reserve(image.data()->size());

  std::copy_n(image.data()->Data(), image.data()->size(), std::back_inserter(ret.data));
  return ret;
}

/*
 * Point
 */
flatbuffers::grpc::Message<seerep::fb::Point> toFlat(const geometry_msgs::Point& point)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(point, builder));
  return builder.ReleaseMessage<seerep::fb::Point>();
}
flatbuffers::Offset<seerep::fb::Point> toFlat(const geometry_msgs::Point& point,
                                              flatbuffers::grpc::MessageBuilder& builder)
{
  seerep::fb::PointBuilder pointbuilder(builder);
  pointbuilder.add_x(point.x);
  pointbuilder.add_y(point.y);
  pointbuilder.add_z(point.z);
  return pointbuilder.Finish();
}

geometry_msgs::Point toROS(const seerep::fb::Point& point)
{
  geometry_msgs::Point ret;
  ret.x = point.x();
  ret.y = point.y();
  ret.z = point.z();
  return ret;
}

/*
 * Quaternion
 */
flatbuffers::grpc::Message<seerep::fb::Quaternion> toFlat(const geometry_msgs::Quaternion& quaternion)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(quaternion, builder));
  return builder.ReleaseMessage<seerep::fb::Quaternion>();
}
flatbuffers::Offset<seerep::fb::Quaternion> toFlat(const geometry_msgs::Quaternion& quaternion,
                                                   flatbuffers::grpc::MessageBuilder& builder)
{
  seerep::fb::QuaternionBuilder quaternionbuilder(builder);
  quaternionbuilder.add_x(quaternion.x);
  quaternionbuilder.add_y(quaternion.y);
  quaternionbuilder.add_z(quaternion.z);
  quaternionbuilder.add_w(quaternion.w);
  return quaternionbuilder.Finish();
}

geometry_msgs::Quaternion toROS(const seerep::fb::Quaternion& quaternion)
{
  geometry_msgs::Quaternion ret;
  ret.x = quaternion.x();
  ret.y = quaternion.y();
  ret.z = quaternion.z();
  ret.w = quaternion.w();
  return ret;
}

/*
 * Pose
 */
flatbuffers::grpc::Message<seerep::fb::Pose> toFlat(const geometry_msgs::Pose& pose)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(pose, builder));
  return builder.ReleaseMessage<seerep::fb::Pose>();
}
flatbuffers::Offset<seerep::fb::Pose> toFlat(const geometry_msgs::Pose& pose, flatbuffers::grpc::MessageBuilder& builder)
{
  auto position = toFlat(pose.position, builder);
  auto orientation = toFlat(pose.orientation, builder);

  seerep::fb::PoseBuilder posebuilder(builder);
  posebuilder.add_position(position);
  posebuilder.add_orientation(orientation);
  return posebuilder.Finish();
}

geometry_msgs::Pose toROS(const seerep::fb::Pose& pose)
{
  geometry_msgs::Pose ret;
  ret.position = toROS(*pose.position());
  ret.orientation = toROS(*pose.orientation());
  return ret;
}

/*
 * PoseStamped
 */
flatbuffers::grpc::Message<seerep::fb::PoseStamped> toFlat(const geometry_msgs::PoseStamped& pose,
                                                           std::string projectuuid)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(pose, projectuuid, builder));
  return builder.ReleaseMessage<seerep::fb::PoseStamped>();
}
flatbuffers::Offset<seerep::fb::PoseStamped> toFlat(const geometry_msgs::PoseStamped& pose, std::string projectuuid,
                                                    flatbuffers::grpc::MessageBuilder& builder)
{
  auto headerOffset = toFlat(pose.header, projectuuid, builder);
  auto poseOffset = toFlat(pose.pose, builder);

  seerep::fb::PoseStampedBuilder posebuilder(builder);
  posebuilder.add_header(headerOffset);
  posebuilder.add_pose(poseOffset);
  return posebuilder.Finish();
}

geometry_msgs::PoseStamped toROS(const seerep::fb::PoseStamped& pose)
{
  geometry_msgs::PoseStamped ret;
  ret.header = toROS(*pose.header());
  ret.pose = toROS(*pose.pose());
  return ret;
}

/*
 * Vector3
 */
flatbuffers::grpc::Message<seerep::fb::Vector3> toFlat(const geometry_msgs::Vector3& vector)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(vector, builder));
  return builder.ReleaseMessage<seerep::fb::Vector3>();
}
flatbuffers::Offset<seerep::fb::Vector3> toFlat(const geometry_msgs::Vector3& vector,
                                                flatbuffers::grpc::MessageBuilder& builder)
{
  seerep::fb::Vector3Builder vector3builder(builder);
  vector3builder.add_x(vector.x);
  vector3builder.add_y(vector.y);
  vector3builder.add_z(vector.z);
  return vector3builder.Finish();
}

geometry_msgs::Vector3 toROS(const seerep::fb::Vector3& vector)
{
  geometry_msgs::Vector3 ret;
  ret.x = vector.x();
  ret.y = vector.y();
  ret.z = vector.z();
  return ret;
}

/*
 * Vector3Stamped
 */
flatbuffers::grpc::Message<seerep::fb::Vector3Stamped> toFlat(const geometry_msgs::Vector3Stamped& vector,
                                                              std::string projectuuid)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(vector, projectuuid, builder));
  return builder.ReleaseMessage<seerep::fb::Vector3Stamped>();
}
flatbuffers::Offset<seerep::fb::Vector3Stamped>
toFlat(const geometry_msgs::Vector3Stamped& vector, std::string projectuuid, flatbuffers::grpc::MessageBuilder& builder)
{
  auto headerOffset = toFlat(vector.header, projectuuid, builder);
  auto vectorOffset = toFlat(vector.vector, builder);

  seerep::fb::Vector3StampedBuilder vector3builder(builder);
  vector3builder.add_header(headerOffset);
  vector3builder.add_vector(vectorOffset);
  return vector3builder.Finish();
}

geometry_msgs::Vector3Stamped toROS(const seerep::fb::Vector3Stamped& vector)
{
  geometry_msgs::Vector3Stamped ret;
  ret.header = toROS(*vector.header());
  ret.vector = toROS(*vector.vector());
  return ret;
}

/*
 * Transform
 */
flatbuffers::grpc::Message<seerep::fb::Transform> toFlat(const geometry_msgs::Transform& transform)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(transform, builder));
  return builder.ReleaseMessage<seerep::fb::Transform>();
}
flatbuffers::Offset<seerep::fb::Transform> toFlat(const geometry_msgs::Transform& transform,
                                                  flatbuffers::grpc::MessageBuilder& builder)
{
  auto translation = toFlat(transform.translation, builder);
  auto rotation = toFlat(transform.rotation, builder);

  seerep::fb::TransformBuilder transformbuilder(builder);
  transformbuilder.add_translation(translation);
  transformbuilder.add_rotation(rotation);
  return transformbuilder.Finish();
}

geometry_msgs::Transform toROS(const seerep::fb::Transform& transform)
{
  geometry_msgs::Transform ret;
  ret.translation = toROS(*transform.translation());
  ret.rotation = toROS(*transform.rotation());
  return ret;
}

/*
 * TransformStamped
 */
flatbuffers::grpc::Message<seerep::fb::TransformStamped> toFlat(const geometry_msgs::TransformStamped& transform,
                                                                std::string projectuuid, const bool isStatic)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(transform, projectuuid, isStatic, builder));
  return builder.ReleaseMessage<seerep::fb::TransformStamped>();
}
flatbuffers::Offset<seerep::fb::TransformStamped> toFlat(const geometry_msgs::TransformStamped& transform,
                                                         std::string projectuuid, const bool isStatic,
                                                         flatbuffers::grpc::MessageBuilder& builder)
{
  auto headerOffset = toFlat(transform.header, projectuuid, builder);
  auto transformOffset = toFlat(transform.transform, builder);
  auto frameIdOffset = builder.CreateString(transform.child_frame_id);

  seerep::fb::TransformStampedBuilder transformbuilder(builder);
  transformbuilder.add_header(headerOffset);
  transformbuilder.add_child_frame_id(frameIdOffset);
  transformbuilder.add_transform(transformOffset);
  transformbuilder.add_is_static(isStatic);
  return transformbuilder.Finish();
}

geometry_msgs::TransformStamped toROS(const seerep::fb::TransformStamped& transform)
{
  geometry_msgs::TransformStamped ret;
  ret.header = toROS(*transform.header());
  ret.child_frame_id = transform.child_frame_id()->str();
  ret.transform = toROS(*transform.transform());
  return ret;
}

/*
 * BoundingBox2DLabeledStamped
 */

flatbuffers::grpc::Message<seerep::fb::BoundingBoxes2DLabeledStamped>
toFlat(const vision_msgs::Detection2DArray& detection2d, std::string projectuuid, std::string category,
       std::string msguuid, std::vector<std::string> labels, std::vector<std::string> instances)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(detection2d, projectuuid, builder, category, msguuid, labels, instances));
  return builder.ReleaseMessage<seerep::fb::BoundingBoxes2DLabeledStamped>();
}
flatbuffers::Offset<seerep::fb::BoundingBoxes2DLabeledStamped>
toFlat(const vision_msgs::Detection2DArray& detection2d, std::string projectuuid,
       flatbuffers::grpc::MessageBuilder& builder, std::string category, std::string msguuid,
       std::vector<std::string> labels, std::vector<std::string> instances)
{
  // convert header
  auto header = toFlat(detection2d.header, projectuuid, builder, msguuid);

  // create boundingbox labeled vector
  std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>> bblabeled;

  if (detection2d.detections.size() == labels.size() && detection2d.detections.size() == instances.size())
  {
    for (long unsigned int i = 0; i < bblabeled.size(); i++)
    {
      bblabeled.push_back(toFlat(detection2d.detections.at(i), builder));
    }
  }
  else
  {
    // for each loop for saving in fb bb_labeled vector
    for (vision_msgs::Detection2D detection : detection2d.detections)
    {
      bblabeled.push_back(toFlat(detection, builder));
    }
  }

  // fb labels vector
  auto labelsOffset = builder.CreateVector(bblabeled);

  auto categoryOffset = builder.CreateString(category);

  seerep::fb::BoundingBox2DLabeledWithCategoryBuilder bbWithCategoryBuilder(builder);
  bbWithCategoryBuilder.add_category(categoryOffset);
  bbWithCategoryBuilder.add_boundingBox2dLabeled(labelsOffset);
  auto bbWithCategoryOffset = bbWithCategoryBuilder.Finish();

  std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>> bbWithCategoryVector;
  bbWithCategoryVector.push_back(bbWithCategoryOffset);
  auto bbWithCatbbWithCategoryVectorOffset = builder.CreateVector(bbWithCategoryVector);

  seerep::fb::BoundingBoxes2DLabeledStampedBuilder bbbuilder(builder);
  bbbuilder.add_header(header);
  bbbuilder.add_labels_bb(bbWithCatbbWithCategoryVectorOffset);
  return bbbuilder.Finish();
}

/*
 * BoundingBox2DLabeled
 */

flatbuffers::grpc::Message<seerep::fb::BoundingBox2DLabeled> toFlat(const vision_msgs::Detection2D& detection2d)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(detection2d, builder));
  return builder.ReleaseMessage<seerep::fb::BoundingBox2DLabeled>();
}
flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled> toFlat(const vision_msgs::Detection2D& detection2d,
                                                             flatbuffers::grpc::MessageBuilder& builder,
                                                             std::string label, std::string instanceUUID)
{
  seerep::fb::Point2DBuilder pointBuilderCenter(builder);
  pointBuilderCenter.add_x(detection2d.bbox.center.x);
  pointBuilderCenter.add_y(detection2d.bbox.center.y);
  auto pointCenter = pointBuilderCenter.Finish();

  seerep::fb::Point2DBuilder pointBuilderSpatialExtent(builder);
  pointBuilderSpatialExtent.add_x(detection2d.bbox.size_x);
  pointBuilderSpatialExtent.add_y(detection2d.bbox.size_y);
  auto pointSpatialExtent = pointBuilderSpatialExtent.Finish();

  seerep::fb::Boundingbox2DBuilder bbbuilder(builder);
  bbbuilder.add_center_point(pointCenter);
  bbbuilder.add_spatial_extent(pointSpatialExtent);
  auto bb = bbbuilder.Finish();

  flatbuffers::Offset<flatbuffers::String> labelOffset;
  if (label == "")
  {
    labelOffset = builder.CreateString(std::to_string(detection2d.results.at(0).id));
  }
  else
  {
    labelOffset = builder.CreateString(label);
  }

  flatbuffers::Offset<flatbuffers::String> InstanceOffset = builder.CreateString(instanceUUID);

  seerep::fb::LabelBuilder labelBuilder(builder);
  labelBuilder.add_label(labelOffset);
  labelBuilder.add_confidence(detection2d.results.at(0).score);
  auto labelMsg = labelBuilder.Finish();

  seerep::fb::LabelWithInstanceBuilder labelInstanceBuilder(builder);
  labelInstanceBuilder.add_instanceUuid(InstanceOffset);
  labelInstanceBuilder.add_label(labelMsg);
  auto labelWithInstanceOffset = labelInstanceBuilder.Finish();

  seerep::fb::BoundingBox2DLabeledBuilder bblabeledbuilder(builder);
  bblabeledbuilder.add_bounding_box(bb);
  bblabeledbuilder.add_labelWithInstance(labelWithInstanceOffset);
  return bblabeledbuilder.Finish();
}

/*
 * camera instrinsic
 */

flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>
toFlat(const sensor_msgs::CameraInfo& ci, std::string& projectuuid, std::string& msgUuid, double maxViewingDistance)
{
  flatbuffers::grpc::MessageBuilder builder;
  builder.Finish(toFlat(ci, projectuuid, msgUuid, maxViewingDistance, builder));
  return builder.ReleaseMessage<seerep::fb::CameraIntrinsics>();
}
flatbuffers::Offset<seerep::fb::CameraIntrinsics> toFlat(const sensor_msgs::CameraInfo& ci, std::string& projectuuid,
                                                         std::string& msgUuid, double maxViewingDistance,
                                                         flatbuffers::grpc::MessageBuilder& builder)
{
  auto header = toFlat(ci.header, projectuuid, builder, msgUuid);

  auto distortionModel = builder.CreateString(ci.distortion_model);

  std::vector<double> distortion;
  for (double d : ci.D)
  {
    distortion.push_back(d);
  }
  auto distortionVector = builder.CreateVector(distortion);

  std::vector<double> intrinsicMatrix;
  for (double i : ci.K)
  {
    intrinsicMatrix.push_back(i);
  }
  auto intrinsicMatrixVector = builder.CreateVector(intrinsicMatrix);

  std::vector<double> projectionMatrix;
  for (double p : ci.P)
  {
    projectionMatrix.push_back(p);
  }
  auto projectionMatrixVector = builder.CreateVector(projectionMatrix);

  std::vector<double> rectificationMatrix;
  for (double r : ci.R)
  {
    rectificationMatrix.push_back(r);
  }
  auto rectificationMatrixVector = builder.CreateVector(rectificationMatrix);

  std::vector<flatbuffers::Offset<seerep::fb::RegionOfInterest>> roiVector;
  auto roi = toFlat(ci.roi, builder);

  seerep::fb::CameraIntrinsicsBuilder cibuilder(builder);
  cibuilder.add_binning_x(ci.binning_x);
  cibuilder.add_binning_y(ci.binning_y);
  cibuilder.add_distortion(distortionVector);
  cibuilder.add_distortion_model(distortionModel);
  cibuilder.add_header(header);
  cibuilder.add_intrinsic_matrix(intrinsicMatrixVector);
  cibuilder.add_maximum_viewing_distance(maxViewingDistance);
  cibuilder.add_projection_matrix(projectionMatrixVector);
  cibuilder.add_rectification_matrix(rectificationMatrixVector);
  cibuilder.add_region_of_interest(roi);
  cibuilder.add_width(ci.width);
  cibuilder.add_height(ci.height);
  return cibuilder.Finish();
}
flatbuffers::Offset<seerep::fb::RegionOfInterest> toFlat(const sensor_msgs::RegionOfInterest& roi,

                                                         flatbuffers::grpc::MessageBuilder& builder)
{
  seerep::fb::RegionOfInterestBuilder roiBuilder(builder);
  roiBuilder.add_do_rectify(roi.do_rectify);
  roiBuilder.add_height(roi.height);
  roiBuilder.add_width(roi.width);
  roiBuilder.add_x_offset(roi.x_offset);
  roiBuilder.add_y_offset(roi.y_offset);
  return roiBuilder.Finish();
}
}  // namespace seerep_ros_conversions_fb
