#ifndef SEEREP_CORE_CORE_INSTANCES_H_
#define SEEREP_CORE_CORE_INSTANCES_H_

#include <functional>
#include <optional>

#include "core-instance.h"

// seerep-msgs
#include <seerep-msgs/label-with-instance.h>
// seerep-hdf5-core
#include <seerep-hdf5-core/hdf5-core-instance.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core
{
class CoreInstances
{
public:
  CoreInstances(std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> hdf5_io);
  ~CoreInstances();

  std::shared_ptr<seerep_core::CoreInstance> createNewInstance();
  std::shared_ptr<seerep_core::CoreInstance> createNewInstance(boost::uuids::uuid uuid);

  std::optional<std::string> getAttribute(const boost::uuids::uuid& uuidInstance, const std::string& key) const;
  void writeAttribute(const boost::uuids::uuid& uuidInstance, const std::string& key, const std::string& value);

  std::vector<boost::uuids::uuid> getImages(const std::vector<boost::uuids::uuid>& instanceIds) const;
  void addImage(const boost::uuids::uuid& uuidInstance, const boost::uuids::uuid& uuidDataset);

private:
  void recreateInstances();
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> m_hdf5_io;
  // map from the uuid of an instance to the object of it
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::CoreInstance>, boost::hash<boost::uuids::uuid>>
      m_instances;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_INSTANCES_H_
