#ifndef SEEREP_CORE_POINTCLOUD_H_
#define SEEREP_CORE_POINTCLOUD_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
// #include <seerep-msgs/point_cloud_2_labeled.pb.h>
// seerep-hdf5
#include <seerep-hdf5/ioPointCloud.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "aabb-hierarchy.h"

#include <Eigen/Dense>

#include <highfive/H5Group.hpp>

namespace seerep_core
{
class Pointcloud
{
public:
  Pointcloud(const std::string& uuid, HighFive::Group& cloud_group);

  ~Pointcloud();

  std::optional<seerep::PointCloud2> getData(const seerep::Query& query);

  void getMinMaxFromBundingBox(Eigen::Vector4f& minPt, Eigen::Vector4f& maxPt, const seerep::Boundingbox& bb);

  AabbHierarchy::AABB getAABB();
  std::string getUUID();

private:
  const std::string m_uuid;
  std::unordered_map<uint64_t, std::shared_ptr<seerep_core::Pointcloud>> m_datasets;

  const HighFive::Group& m_cloud_group;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_POINTCLOUD_H_
