#ifndef SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_

// seerep-msgs
#include <seerep_msgs/dataset_indexable.h>

// std
#include <boost/uuid/uuid.hpp>
#include <optional>

namespace seerep_hdf5_core
{
class Hdf5CoreDatatypeInterface
{
public:
  virtual std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const boost::uuids::uuid& uuid) = 0;
  virtual std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const std::string& uuid) = 0;

  virtual std::vector<std::string> getDatasetUuids() = 0;
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_ */
