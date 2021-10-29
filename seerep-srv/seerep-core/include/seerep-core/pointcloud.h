#ifndef SEEREP_CORE_POINTCLOUD_H_
#define SEEREP_CORE_POINTCLOUD_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/boundingbox.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
#include <seerep-msgs/point_cloud_2_labeled.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "aabb-hierarchy.h"

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/moment_of_inertia_estimation.h>

namespace seerep_core
{
class Pointcloud
{
public:
  Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
             const seerep::PointCloud2& pointcloud2, const uint64_t& id);
  Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const uint64_t& id);
  // labeled
  Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
             const seerep::PointCloud2Labeled& pointcloud2, const uint64_t& id);
  ~Pointcloud();

  std::optional<seerep::PointCloud2> getData(const seerep::Boundingbox bb);

  void getMinMaxFromBundingBox(Eigen::Vector4f& minPt, Eigen::Vector4f& maxPt, const seerep::Boundingbox& bb);

  void protoToPcl(const seerep::PointCloud2& pc_proto, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl);
  void pclToProto(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl, seerep::PointCloud2& pc_proto);

  AabbHierarchy::AABB getAABB();
  uint64_t getID();

private:
  std::string m_coordinatesystem;
  std::string m_coordinatesystemParent;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;
  const uint64_t m_id;
  // axis aligned bounding box
  AabbHierarchy::AABB m_aabb;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_POINTCLOUD_H_
