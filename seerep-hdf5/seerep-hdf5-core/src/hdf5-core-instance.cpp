#include "seerep-hdf5-core/hdf5-core-instance.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CoreInstance::Hdf5CoreInstance(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

std::optional<std::unordered_map<std::string, std::string>>
Hdf5CoreInstance::readAttributes(const boost::uuids::uuid& uuid)
{
  return Hdf5CoreInstance::readAttributes(boost::lexical_cast<std::string>(uuid));
}

std::optional<std::unordered_map<std::string, std::string>> Hdf5CoreInstance::readAttributes(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = HDF5_GROUP_INSTANCE + "/" + uuid;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  if (!m_file->exist(hdf5GroupPath) || !m_file->exist(hdf5DatasetTimePath) || !m_file->exist(hdf5DatasetTransPath) ||
      !m_file->exist(hdf5DatasetRotPath))
  {
    return std::nullopt;
  }
}

void Hdf5CoreInstance::writeAttribute(const boost::uuids::uuid& uuid, std::string key, std::string value)
{
  writeAttribute(boost::lexical_cast<std::string>(uuid), key, value);
}

void Hdf5CoreInstance::writeAttribute(const std::string& uuid, std::string key, std::string value)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = HDF5_GROUP_INSTANCE + "/" + uuid;
}

}  // namespace seerep_hdf5_core
