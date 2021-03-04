#include "agri_gaia_ros/conversions.h"

namespace agri_gaia_ros
{
  /*
   * Header
   */
  ag::ros::Header toProto(const std_msgs::Header& header)
  {
    ag::ros::Header ret;
    ret.set_seq(header.seq);
    ret.set_frame_id(header.frame_id);
    ret.mutable_stamp()->set_seconds(header.stamp.sec);
    ret.mutable_stamp()->set_nanos(header.stamp.nsec);
    return ret;
  }

  std_msgs::Header toROS(const ag::ros::Header& header)
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
  ag::ros::PointField toProto(const sensor_msgs::PointField& point_field)
  {
    ag::ros::PointField ret;
    ret.set_name(point_field.name);
    ret.set_offset(point_field.offset);
    ret.set_datatype(ag::ros::PointField_Datatype(point_field.datatype));
    ret.set_count(point_field.count);
    return ret;
  }

  sensor_msgs::PointField toROS(const ag::ros::PointField& point_field)
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
  ag::ros::PointCloud2 toProto(const sensor_msgs::PointCloud2& cloud)
  {
    ag::ros::PointCloud2 ret;
    *ret.mutable_header() = toProto(cloud.header);
    ret.set_height(cloud.height);
    ret.set_width(cloud.width);
    for(auto field : cloud.fields)
      *ret.add_fields() = toProto(field);
    ret.set_is_bigendian(cloud.is_bigendian);
    ret.set_point_step(cloud.point_step);
    ret.set_row_step(cloud.row_step);
    ret.set_data(&cloud.data.front(), cloud.data.size());
    ret.set_is_dense(cloud.is_dense);
    return ret;
  }

  sensor_msgs::PointCloud2 toROS(const ag::ros::PointCloud2& cloud)
  {
    sensor_msgs::PointCloud2 ret;
    ret.header = toROS(cloud.header());
    ret.height = cloud.height();
    ret.width = cloud.width();
    for(auto field: cloud.fields())
      ret.fields.push_back(toROS(field));
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
  ag::ros::Image toProto(const sensor_msgs::Image& image)
  {
    ag::ros::Image ret;
    *ret.mutable_header() = toProto(image.header);
    ret.set_height(image.height);
    ret.set_width(image.width);
    ret.set_encoding(image.encoding);
    ret.set_is_bigendian(image.is_bigendian);
    ret.set_step(image.step);
    ret.set_data(&image.data.front(), image.data.size());
    return ret;
  }

  sensor_msgs::Image toROS(const ag::ros::Image& image)
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

} /* namespace agri_gaia_ros */
