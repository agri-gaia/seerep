#ifndef SEEREP_HDF5_IO_IMAGE_CORE_H_
#define SEEREP_HDF5_IO_IMAGE_CORE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "seerep-core-io/general-io-core.h"

// seerep-msgs
#include <seerep-msgs/dataset-indexable.h>

// std
#include <optional>

#include <boost/geometry.hpp>

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_core_io
{
class ImageIOCore : public GeneralIOCore
{
public:
  ImageIOCore(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<seerep_core_msgs::DatasetIndexable> readImage(const boost::uuids::uuid& uuid);

private:
  const std::string SIZE = "size";
  const std::string CLASS = "CLASS";

  // image / pointcloud attribute keys
  inline static const std::string HEIGHT = "height";
  inline static const std::string WIDTH = "width";
  inline static const std::string ENCODING = "encoding";
  inline static const std::string IS_BIGENDIAN = "is_bigendian";
  inline static const std::string ROW_STEP = "row_step";
  inline static const std::string POINT_STEP = "point_step";
  inline static const std::string IS_DENSE = "is_dense";

  inline static const std::string RAWDATA = "rawdata";

public:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_IMAGE = "images";
};

}  // namespace seerep_core_io

#endif /* SEEREP_HDF5_IO_IMAGE_CORE_H_ */
