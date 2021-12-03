#ifndef SEEREP_CORE_POINTCLOUD_OVERVIEW_H_
#define SEEREP_CORE_POINTCLOUD_OVERVIEW_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
#include <seerep-msgs/point_cloud_2_labeled.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "pointcloud.h"
#include "aabb-hierarchy.h"

namespace seerep_core
{
class PointcloudOverview
{
public:
  PointcloudOverview();
  PointcloudOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io);
  ~PointcloudOverview();
  std::vector<std::optional<seerep::PointCloud2>> getData(const seerep::Query& query);

  void addDataset(const seerep::PointCloud2& pointcloud2);
  void addDatasetLabeled(const seerep::PointCloud2Labeled& pointcloud2labeled);

private:
  void recreateDatasets();

  uint64_t data_count;

  std::string coordinatesystem;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;

  std::unordered_map<uint64_t, std::shared_ptr<seerep_core::Pointcloud>> m_datasets;

  AabbHierarchy::rtree m_rt;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_POINTCLOUD_OVERVIEW_H_
