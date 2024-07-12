#include "seerep_core/core_instances.h"

namespace seerep_core
{
CoreInstances::CoreInstances(std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> hdf5_io) : m_hdf5_io(hdf5_io)
{
  recreateInstances();
}
CoreInstances::~CoreInstances()
{
}

std::shared_ptr<seerep_core::CoreInstance> CoreInstances::createNewInstance(const std::string& label)
{
  return createNewInstance(seerep_core_msgs::Label{ .label = label,
                                                    .labelIdDatumaro = 0,
                                                    .uuidInstance = boost::uuids::random_generator()(),
                                                    .instanceIdDatumaro = 0 });
}
std::shared_ptr<seerep_core::CoreInstance> CoreInstances::createNewInstance(const seerep_core_msgs::Label& label)
{
  auto instance = std::make_shared<seerep_core::CoreInstance>(seerep_core::CoreInstance(m_hdf5_io, label.uuidInstance));

  instance->writeAttribute(CoreInstances::ATTRIBUTELABEL, label.label);
  m_instances.emplace(label.uuidInstance, instance);
  return instance;
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

std::vector<boost::uuids::uuid> CoreInstances::getDatasets(const std::vector<boost::uuids::uuid>& instanceIds,
                                                           const seerep_core_msgs::Datatype& datatype) const
{
  std::vector<boost::uuids::uuid> imagesAll;

  for (auto instanceId : instanceIds)
  {
    auto instance = m_instances.find(instanceId);

    if (instance != m_instances.end())
    {
      auto img = instance->second->getDatasets(datatype);
      imagesAll.insert(std::end(imagesAll), std::begin(img), std::end(img));
    }
  }

  return imagesAll;
}
void CoreInstances::addDataset(const seerep_core_msgs::Label& label, const boost::uuids::uuid& uuidDataset,
                               const seerep_core_msgs::Datatype& datatype)
{
  auto instanceMapEntry = m_instances.find(label.uuidInstance);
  std::shared_ptr<seerep_core::CoreInstance> instance;
  if (instanceMapEntry == m_instances.end())
  {
    instance = createNewInstance(label);
  }
  else
  {
    instance = m_instances.at(label.uuidInstance);
  }
  /// only add the dataset to an existing instance if the labels match otherwise throw an error
  if (instance->getAttribute(CoreInstances::ATTRIBUTELABEL) == label.label)
  {
    /// TODO: add categories to instances!
    instance->addDataset(uuidDataset, datatype);
  }
  else
  {
    throw std::runtime_error("label of instance and of new dataset do not match!");
  }
}

void CoreInstances::recreateInstances()
{
  std::vector<std::string> instances =
      m_hdf5_io->getGroupDatasets(seerep_hdf5_core::Hdf5CoreInstance::HDF5_GROUP_INSTANCE);
  for (auto const& name : instances)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "found " << name << " in HDF5 file.";

    try
    {
      boost::uuids::string_generator gen;
      boost::uuids::uuid uuid = gen(name);
      m_instances.emplace(uuid, std::make_shared<seerep_core::CoreInstance>(seerep_core::CoreInstance(m_hdf5_io, uuid)));
    }
    catch (const std::runtime_error& e)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    }
  }
}

} /* namespace seerep_core */
