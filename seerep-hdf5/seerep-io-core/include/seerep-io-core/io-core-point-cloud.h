#ifndef SEEREP_IO_CORE_IO_CORE_POINT_CLOUD_H_
#define SEEREP_IO_CORE_IO_CORE_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "seerep-io-core/io-core-general.h"

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

namespace seerep_io_core
{
class IoCorePointCloud : public IoCoreGeneral
{
public:
  IoCorePointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<seerep_core_msgs::DatasetIndexable> readPointCloud(const boost::uuids::uuid& uuid);

public:
  // image / pointcloud attribute keys
  inline static const std::string HEIGHT = "height";
  inline static const std::string WIDTH = "width";
  inline static const std::string ENCODING = "encoding";
  inline static const std::string IS_BIGENDIAN = "is_bigendian";
  inline static const std::string ROW_STEP = "row_step";
  inline static const std::string POINT_STEP = "point_step";
  inline static const std::string IS_DENSE = "is_dense";

  // pointcloud fields attribute keys
  inline static const std::string FIELD_NAME = "field_name_";
  inline static const std::string FIELD_OFFSET = "field_offset_";
  inline static const std::string FIELD_DATATYPE = "field_datatype_";
  inline static const std::string FIELD_COUNT = "field_count_";

  // point and quaternion attribute keys
  const std::string X = "x";
  const std::string Y = "y";
  const std::string Z = "z";
  const std::string W = "w";

  // make private again after fixing io calls of pointcloud.cpp and pointcloud-overview.cpp
  inline static const std::string BOUNDINGBOX = "boundingbox";
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_POINTCLOUD = "pointclouds";
};

}  // namespace seerep_io_core

#endif /* SEEREP_IO_CORE_IO_CORE_POINT_CLOUD_H_ */
