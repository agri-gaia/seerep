#include "seerep-core-io/image-io-core.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_core_io
{
ImageIOCore::ImageIOCore(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : GeneralIOCore(file, write_mtx)
{
}

std::optional<seerep_core_msgs::DatasetIndexable> ImageIOCore::readImage(const boost::uuids::uuid& uuid)
{
  std::string id = boost::lexical_cast<std::string>(uuid);
  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;

  if (!m_file->exist(hdf5DatasetPath))
    return std::nullopt;

  return readDataForIndices(hdf5DatasetPath, id);
}

}  // namespace seerep_core_io
