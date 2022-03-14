#include "seerep-hdf5-core/hdf5-core-image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CoreImage::Hdf5CoreImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

std::optional<seerep_core_msgs::DatasetIndexable> Hdf5CoreImage::readImage(const boost::uuids::uuid& uuid)
{
  std::string id = boost::lexical_cast<std::string>(uuid);
  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;

  if (!m_file->exist(hdf5DatasetPath))
    return std::nullopt;

  return readDataForIndices(hdf5DatasetPath, id);
}

}  // namespace seerep_hdf5_core
