#ifndef SEEREP_CORE_POINTCLOUD_OVERVIEW_H_
#define SEEREP_CORE_POINTCLOUD_OVERVIEW_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/boundingbox.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "pointcloud.h"

namespace seerep_core
{
class PointcloudOverview
{
public:
  PointcloudOverview(std::string projectname);
  ~PointcloudOverview();
  std::vector<std::unique_ptr<seerep::PointCloud2>> getData(const std::string& id, const seerep::Boundingbox& bb);

  void addDataset(const std::string& id, const seerep::PointCloud2& pointcloud2);

private:
  void recreateDatasets();

  std::string projectname;
  std::string coordinatesystem;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io;

  std::unordered_map<std::string, std::shared_ptr<seerep_core::Pointcloud>> datasets;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_POINTCLOUD_OVERVIEW_H_
