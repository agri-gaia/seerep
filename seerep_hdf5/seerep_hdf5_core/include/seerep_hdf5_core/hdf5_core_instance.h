#ifndef SEEREP_HDF5_CORE_HDF5_CORE_INSTANCE_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_INSTANCE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5_core
#include "hdf5_core_general.h"

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_core
{
class Hdf5CoreInstance : public Hdf5CoreGeneral
{
public:
  Hdf5CoreInstance(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::unordered_map<std::string, std::string>> readAttributes(const boost::uuids::uuid& uuid);
  std::optional<std::unordered_map<std::string, std::string>> readAttributes(const std::string& uuid);
  void writeAttribute(const boost::uuids::uuid& uuid, std::string key, std::string value);
  void writeAttribute(const std::string& uuid, std::string key, std::string value);

public:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_INSTANCE = "instance";
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_INSTANCE_H_ */
