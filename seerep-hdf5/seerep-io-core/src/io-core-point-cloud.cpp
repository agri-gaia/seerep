#include "seerep-io-core/io-core-point-cloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_io_core
{
IoCorePointCloud::IoCorePointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : IoCoreGeneral(file, write_mtx)
{
}

std::optional<seerep_core_msgs::DatasetIndexable> IoCorePointCloud::readPointCloud(const boost::uuids::uuid& uuid)
{
  std::string id = boost::lexical_cast<std::string>(uuid);
  std::string hdf5DatasetPath = HDF5_GROUP_POINTCLOUD + "/" + id;

  if (!m_file->exist(hdf5DatasetPath))
    return std::nullopt;

  return readDataForIndices(hdf5DatasetPath, id);
}

}  // namespace seerep_io_core
