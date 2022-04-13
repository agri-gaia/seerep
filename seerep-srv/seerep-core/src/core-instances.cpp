#include "seerep-core/core-instances.h"

namespace seerep_core
{
CoreInstances::CoreInstances(std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> hdf5_io) : m_hdf5_io(hdf5_io)
{
  recreateInstances();
}
CoreInstances::~CoreInstances()
{
}

void CoreInstances::recreateInstances()
{
  std::vector<std::string> instances =
      m_hdf5_io->getGroupDatasets(seerep_hdf5_core::Hdf5CoreInstance::HDF5_GROUP_INSTANCE);
  for (auto const& name : instances)
  {
    std::cout << "found " << name << " in HDF5 file." << std::endl;

    try
    {
      boost::uuids::string_generator gen;
      boost::uuids::uuid uuid = gen(name);
      m_instances.emplace(uuid, std::make_shared<seerep_core::CoreInstance>(seerep_core::CoreInstance(m_hdf5_io, uuid)));
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

boost::uuids::uuid CoreInstances::createNewInstance()
{
  auto uuid = boost::uuids::random_generator()();
  m_instances.emplace(uuid, std::make_shared<seerep_core::CoreInstance>(seerep_core::CoreInstance(m_hdf5_io, uuid)));

  return uuid;
}

std::vector<boost::uuids::uuid> CoreInstances::getImages(const std::vector<boost::uuids::uuid>& instanceIds) const
{
  std::vector<boost::uuids::uuid> imagesAll;

  for (auto instanceId : instanceIds)
  {
    auto instance = m_instances.find(instanceId);

    if (instance != m_instances.end())
    {
      auto img = instance->second->getImages();
      imagesAll.insert(std::end(imagesAll), std::begin(img), std::end(img));
    }
  }

  return imagesAll;
}

void CoreInstances::addImage(const boost::uuids::uuid& uuidInstance, const boost::uuids::uuid& uuidDataset)
{
  // TODO check if instances exists
  m_instances.at(uuidInstance)->addImage(uuidDataset);
}

std::optional<std::string> CoreInstances::getAttribute(const boost::uuids::uuid& uuidInstance,
                                                       const std::string& key) const
{
  auto instance = m_instances.find(uuidInstance);

  if (instance != m_instances.end())
  {
    return m_instances.at(uuidInstance)->getAttribute(key);
  }
  else
  {
    return std::nullopt;
  }
}
void CoreInstances::writeAttribute(const boost::uuids::uuid& uuidInstance, const std::string& key,
                                   const std::string& value)
{
  auto instance = m_instances.find(uuidInstance);

  if (instance != m_instances.end())
  {
    m_instances.at(uuidInstance)->writeAttribute(key, value);
  }
  else
  {
    throw std::runtime_error("there is no instance with id " + boost::lexical_cast<std::string>(uuidInstance));
  }
}

} /* namespace seerep_core */
