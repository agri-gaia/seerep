#ifndef SEEREP_HDF5_CORE_HDF5_CORE_POINT_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_POINT_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include "seerep-hdf5-core/hdf5-core-datatype-interface.h"
#include "seerep-hdf5-core/hdf5-core-general.h"

// seerep-msgs
#include <seerep-msgs/dataset-indexable.h>

// std
#include <boost/geometry.hpp>
#include <optional>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_hdf5_core
{
class Hdf5CorePoint : public Hdf5CoreGeneral, public Hdf5CoreDatatypeInterface
{
public:
  Hdf5CorePoint(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const boost::uuids::uuid& uuid);
  std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const std::string& uuid);

  std::vector<std::string> getDatasetUuids();

private:
  std::vector<std::string> readLabelsGeneral(const std::string& dataGroup);
  std::vector<std::string> readBoundingBoxLabels(const std::string& dataGroup);

public:
  inline static const std::string RAWDATA = "rawdata";

  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_POINT = "points";
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_POINT_H_ */
