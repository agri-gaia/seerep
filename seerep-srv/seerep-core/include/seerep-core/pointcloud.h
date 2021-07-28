#ifndef SEEREP_CORE_POINTCLOUD_H_
#define SEEREP_CORE_POINTCLOUD_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/boundingbox.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>

#include <pcl_conversions/pcl_conversions.h>

namespace seerep_core
{
class Pointcloud
{
public:
  Pointcloud();
  ~Pointcloud();
  static std::optional<seerep::PointCloud2> getData(seerep_hdf5::SeerepHDF5IO& hdf5_io, const std::string& id,
                                                    const seerep::Boundingbox bb);

  static void getBoundingBox(Eigen::Vector4f& minPt, Eigen::Vector4f& maxPt, const seerep::Boundingbox& bb);
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_POINTCLOUD_H_
