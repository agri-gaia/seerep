#include "seerep-core/core-instance.h"

namespace seerep_core
{
CoreInstance::CoreInstance(std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> hdf5_io,
                           const boost::uuids::uuid& uuidInstance)
  : m_hdf5_io(hdf5_io), m_uuid(uuidInstance)
{
  recreateInstances();
}
CoreInstance::~CoreInstance()
{
}

boost::uuids::uuid CoreInstance::getUUID()
{
  return m_uuid;
}

std::optional<std::string> CoreInstance::getAttribute(const std::string& key) const
{
  auto value = m_attributes.find(key);

  if (value != m_attributes.end())
  {
    return value->second;
  }
  else
  {
    return std::nullopt;
  }
}

void CoreInstance::writeAttribute(const std::string& key, const std::string& value)
{
  auto emplaceResult = m_attributes.emplace(key, value);

  // key already in map -> override!
  if (!emplaceResult.second)
  {
    m_attributes.at(key) = value;
  }

  m_hdf5_io->writeAttribute(m_uuid, key, value);
}

std::vector<boost::uuids::uuid> CoreInstance::getImages() const
{
  return m_images;
}

void CoreInstance::addImage(const boost::uuids::uuid& uuidDataset)
{
  m_images.push_back(uuidDataset);
}

void CoreInstance::recreateInstances()
{
  auto attributes = m_hdf5_io->readAttributes(m_uuid);

  if (attributes)
  {
    m_attributes = std::move(*attributes);
  }
}
} /* namespace seerep_core */
