#ifndef SEEREP_CORE_CORE_INSTANCE_H_
#define SEEREP_CORE_CORE_INSTANCE_H_

#include <functional>
#include <optional>

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
class CoreInstance
{
public:
  CoreInstance(std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> hdf5_io, boost::uuids::uuid& uuid);
  ~CoreInstance();

  std::vector<boost::uuids::uuid> getImages() const;
  void addImage(const boost::uuids::uuid& uuidDataset);

  std::optional<std::string> getAttribute(const std::string& key) const;
  void writeAttribute(const std::string& key, const std::string& value);

private:
  void recreateInstances();
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> m_hdf5_io;

  boost::uuids::uuid m_uuid;

  std::unordered_map<std::string, std::string> m_attributes;

  // images
  std::vector<boost::uuids::uuid> m_images;
  // point clouds
  std::vector<boost::uuids::uuid> m_pointClouds;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_INSTANCE_H_
