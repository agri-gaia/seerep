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

std::shared_ptr<seerep_core::CoreInstance> CoreInstances::createNewInstance(const std::string& label)
{
  return createNewInstance(
      seerep_core_msgs::LabelWithInstance{ .label = label, .uuidInstance = boost::uuids::random_generator()() });
}
std::shared_ptr<seerep_core::CoreInstance>
CoreInstances::createNewInstance(const seerep_core_msgs::LabelWithInstance& labelWithInstance)
{
  auto instance =
      std::make_shared<seerep_core::CoreInstance>(seerep_core::CoreInstance(m_hdf5_io, labelWithInstance.uuidInstance));
  /// @todo set label in instance
  // instace->setLabel(labelWithInstance.label);
  m_instances.emplace(labelWithInstance.uuidInstance, instance);
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
void CoreInstances::addImage(const seerep_core_msgs::LabelWithInstance& labelWithInstance,
                             const boost::uuids::uuid& uuidDataset)
{
  auto instanceMapEntry = m_instances.find(labelWithInstance.uuidInstance);
  std::shared_ptr<seerep_core::CoreInstance> instance;
  if (instanceMapEntry == m_instances.end())
  {
    instance = createNewInstance(labelWithInstance);
  }
  else
  {
    instance = m_instances.at(labelWithInstance.uuidInstance);
  }
  /// @todo check if label of instance and of dataset match
  // if(instance->getLabel() == labelWithInstance.label){
  instance->addImage(uuidDataset);
  //}
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

} /* namespace seerep_core */
