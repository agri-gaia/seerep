#include "seerep-core/project.h"

namespace seerep_core
{
Project::Project(boost::uuids::uuid& uuid, std::string path) : id(uuid)
{
  coordinatesystem = "test";
  HighFive::File hdf5_file(path, HighFive::File::ReadWrite | HighFive::File::Create);
  // HighFive::File::ReadOnly);
  // HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  hdf5_io = std::make_shared<seerep_hdf5::SeerepHDF5IO>(hdf5_file);
  recreateDatatypes();
}
Project::~Project()
{
}

// std::vector<std::optional<seerep::PointCloud2>> Project::getPointCloud(const seerep::Boundingbox& bb)
// {
// }

void Project::recreateDatatypes()
{
}

void Project::addPointCloud(const seerep::PointCloud2& pointcloud2)
{
}

} /* namespace seerep_core */
