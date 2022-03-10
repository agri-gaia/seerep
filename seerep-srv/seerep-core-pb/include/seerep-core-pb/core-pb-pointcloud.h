#ifndef SEEREP_CORE_PB_POINTCLOUD_H_
#define SEEREP_CORE_PB_POINTCLOUD_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>

#include <seerep-msgs/aabb.h>

// seerep-pb-io
#include <seerep-io-pb/io-pb-pointcloud.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

#include <eigen3/Eigen/Dense>

#include <highfive/H5Group.hpp>

namespace seerep_core_pb
{
class CorePbPointCloud
{
public:
  CorePbPointCloud(const std::string& uuid, HighFive::Group& cloud_group);

  ~CorePbPointCloud();

  std::optional<seerep::PointCloud2> getData(const seerep::Query& query);

  void getMinMaxFromBundingBox(Eigen::Vector4f& minPt, Eigen::Vector4f& maxPt, const seerep::Boundingbox& bb);

  seerep_core_msgs::AABB getAABB();
  std::string getUUID();

private:
  const std::string m_uuid;
  std::unordered_map<uint64_t, std::shared_ptr<seerep_core_pb::CorePbPointCloud>> m_datasets;

  const HighFive::Group& m_cloud_group;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_POINTCLOUD_H_
