#include "seerep-core/pointcloud.h"

namespace seerep_core
{
Pointcloud::Pointcloud()
{
}
Pointcloud::~Pointcloud()
{
}
std::optional<seerep::PointCloud2> Pointcloud::getData(seerep_hdf5::SeerepHDF5IO& hdf5_io, const std::string& id,
                                                       const seerep::Boundingbox bb)
{
  Pointcloud::BoundingBox BoundingBox = getBoundingBox(bb);
  std::optional<seerep::PointCloud2> pc = hdf5_io.readPointCloud2(id);

  if (pc)
  {
    pc.value().data();

    return pc;
  }

  return std::nullopt;
}

Pointcloud::BoundingBox Pointcloud::getBoundingBox(const seerep::Boundingbox bb)
{
  Pointcloud::BoundingBox boundingbox;

  if (bb.point_max().x() < bb.point_min().x())
  {
    boundingbox.xmax = bb.point_min().x();
    boundingbox.xmin = bb.point_max().x();
  }
  else
  {
    boundingbox.xmax = bb.point_max().x();
    boundingbox.xmin = bb.point_min().x();
  }
  if (bb.point_max().y() < bb.point_min().y())
  {
    boundingbox.xmax = bb.point_min().y();
    boundingbox.xmin = bb.point_max().y();
  }
  else
  {
    boundingbox.xmax = bb.point_max().y();
    boundingbox.xmin = bb.point_min().y();
  }
  if (bb.point_max().z() < bb.point_min().z())
  {
    boundingbox.xmax = bb.point_min().z();
    boundingbox.xmin = bb.point_max().z();
  }
  else
  {
    boundingbox.xmax = bb.point_max().z();
    boundingbox.xmin = bb.point_min().z();
  }

  return boundingbox;
}
} /* namespace seerep_core */
