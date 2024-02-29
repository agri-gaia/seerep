#include "seerep_ros_conversions_pb/conversions.h"

namespace seerep_ros_conversions_pb
{
/*
 * Header
 */
seerep::pb::Header toProto(const std_msgs::Header& header)
{
  seerep::pb::Header ret;
  ret.set_seq(header.seq);
  ret.set_frame_id(header.frame_id);
  ret.mutable_stamp()->set_seconds(header.stamp.sec);
  ret.mutable_stamp()->set_nanos(header.stamp.nsec);
  return ret;
}

std_msgs::Header toROS(const seerep::pb::Header& header)
{
  std_msgs::Header ret;
  ret.seq = header.seq();
  ret.frame_id = header.frame_id();
  ret.stamp.sec = header.stamp().seconds();
  ret.stamp.nsec = header.stamp().nanos();
  return ret;
}

/*
 * PointField
 */
seerep::pb::PointField toProto(const sensor_msgs::PointField& point_field)
{
  seerep::pb::PointField ret;
  ret.set_name(point_field.name);
  ret.set_offset(point_field.offset);
  ret.set_datatype(seerep::pb::PointField_Datatype(point_field.datatype));
  ret.set_count(point_field.count);
  return ret;
}

sensor_msgs::PointField toROS(const seerep::pb::PointField& point_field)
{
  sensor_msgs::PointField ret;
  ret.name = point_field.name();
  ret.offset = point_field.offset();
  ret.datatype = point_field.datatype();
  ret.count = point_field.count();
  return ret;
}

/*
 * PointCloud2
 */
seerep::pb::PointCloud2 toProto(const sensor_msgs::PointCloud2& cloud, std::string projectuuid)
{
  seerep::pb::PointCloud2 ret;
  *ret.mutable_header() = toProto(cloud.header);
  *ret.mutable_header()->mutable_uuid_project() = projectuuid;
  ret.set_height(cloud.height);
  ret.set_width(cloud.width);
  for (auto field : cloud.fields)
  {
    *ret.add_fields() = toProto(field);
  }
  ret.set_is_bigendian(cloud.is_bigendian);
  ret.set_point_step(cloud.point_step);
  ret.set_row_step(cloud.row_step);
  ret.set_data(&cloud.data.front(), cloud.data.size());
  ret.set_is_dense(cloud.is_dense);
  return ret;
}

sensor_msgs::PointCloud2 toROS(const seerep::pb::PointCloud2& cloud)
{
  sensor_msgs::PointCloud2 ret;
  ret.header = toROS(cloud.header());
  ret.height = cloud.height();
  ret.width = cloud.width();
  for (auto field : cloud.fields())
  {
    ret.fields.push_back(toROS(field));
  }
  ret.is_bigendian = cloud.is_bigendian();
  ret.point_step = cloud.point_step();
  ret.row_step = cloud.row_step();
  std::copy(cloud.data().begin(), cloud.data().end(), std::back_inserter(ret.data));
  ret.is_dense = cloud.is_dense();
  return ret;
}

/*
 * Image
 */
seerep::pb::Image toProto(const sensor_msgs::Image& image, std::string projectuuid)
{
  seerep::pb::Image ret;
  *ret.mutable_header() = toProto(image.header);
  *ret.mutable_header()->mutable_uuid_project() = projectuuid;
  ret.set_height(image.height);
  ret.set_width(image.width);
  ret.set_encoding(image.encoding);
  ret.set_is_bigendian(image.is_bigendian);
  ret.set_step(image.step);
  ret.set_data(&image.data.front(), image.data.size());
  return ret;
}

sensor_msgs::Image toROS(const seerep::pb::Image& image)
{
  sensor_msgs::Image ret;
  ret.header = toROS(image.header());
  ret.height = image.height();
  ret.width = image.width();
  ret.encoding = image.encoding();
  ret.is_bigendian = image.is_bigendian();
  ret.step = image.step();
  std::copy(image.data().begin(), image.data().end(), std::back_inserter(ret.data));
  return ret;
}

/*
 * Point
 */
seerep::pb::Point toProto(const geometry_msgs::Point& point)
{
  seerep::pb::Point ret;
  ret.set_x(point.x);
  ret.set_y(point.y);
  ret.set_z(point.z);
  return ret;
}

geometry_msgs::Point toROS(const seerep::pb::Point& point)
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
seerep::pb::Quaternion toProto(const geometry_msgs::Quaternion& quaternion)
{
  seerep::pb::Quaternion ret;
  ret.set_x(quaternion.x);
  ret.set_y(quaternion.y);
  ret.set_z(quaternion.z);
  ret.set_w(quaternion.w);
  return ret;
}

geometry_msgs::Quaternion toROS(const seerep::pb::Quaternion& quaternion)
{
  geometry_msgs::Quaternion ret;
  ret.x = quaternion.x();
  ret.y = quaternion.y();
  ret.z = quaternion.z();
  ret.w = quaternion.w();
  return ret;
}

/*
 * Vector3
 */
seerep::pb::Vector3 toProto(const geometry_msgs::Vector3& vector)
{
  seerep::pb::Vector3 ret;
  ret.set_x(vector.x);
  ret.set_y(vector.y);
  ret.set_z(vector.z);
  return ret;
}

geometry_msgs::Vector3 toROS(const seerep::pb::Vector3& vector)
{
  geometry_msgs::Vector3 ret;
  ret.x = vector.x();
  ret.y = vector.y();
  ret.z = vector.z();
  return ret;
}

/*
 * Transform
 */
seerep::pb::Transform toProto(const geometry_msgs::Transform& transform)
{
  seerep::pb::Transform ret;
  *ret.mutable_translation() = toProto(transform.translation);
  *ret.mutable_rotation() = toProto(transform.rotation);
  return ret;
}

geometry_msgs::Transform toROS(const seerep::pb::Transform& transform)
{
  geometry_msgs::Transform ret;
  ret.translation = toROS(transform.translation());
  ret.rotation = toROS(transform.rotation());
  return ret;
}

/*
 * TransformStamped
 */
seerep::pb::TransformStamped toProto(const geometry_msgs::TransformStamped& transform, const bool isStatic,
                                     std::string projectuuid)
{
  seerep::pb::TransformStamped ret;
  *ret.mutable_header() = toProto(transform.header);
  *ret.mutable_header()->mutable_uuid_project() = projectuuid;
  *ret.mutable_child_frame_id() = transform.child_frame_id;
  *ret.mutable_header()->mutable_uuid_project() = projectuuid;
  *ret.mutable_transform() = toProto(transform.transform);
  ret.set_is_static(isStatic);
  return ret;
}

geometry_msgs::TransformStamped toROS(const seerep::pb::TransformStamped& transform)
{
  geometry_msgs::TransformStamped ret;
  ret.header = toROS(transform.header());
  ret.child_frame_id = transform.child_frame_id();
  ret.transform = toROS(transform.transform());
  return ret;
}
}  // namespace seerep_ros_conversions_pb
