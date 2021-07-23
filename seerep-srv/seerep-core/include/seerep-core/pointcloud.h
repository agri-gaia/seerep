#ifndef SEEREP_CORE_POINTCLOUD_H_
#define SEEREP_CORE_POINTCLOUD_H_

#include <functional>
#include <optional>

#include <seerep-msgs/boundingbox.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>

#include <seerep-hdf5/io.h>

namespace seerep_core
{
class Pointcloud
{
public:
  Pointcloud();
  ~Pointcloud();
  std::optional<seerep::PointCloud2> getData(seerep_hdf5::SeerepHDF5IO& hdf5_io, const std::string& id,
                                             const seerep::Boundingbox bb);

private:
  struct BoundingBox
  {
    double xmax, xmin, ymax, ymin, zmax, zmin;
  };
  BoundingBox getBoundingBox(const seerep::Boundingbox bb);
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_POINTCLOUD_H_
