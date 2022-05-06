#ifndef SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_

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
class Hdf5CorePointCloud : public Hdf5CoreGeneral, public Hdf5CoreDatatypeInterface
{
public:
  Hdf5CorePointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const boost::uuids::uuid& uuid);
  std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const std::string& uuid);

  std::vector<std::string> getDatasetUuids();

private:
  std::vector<std::string> readLabelsGeneral(const std::string& dataGroup);
  std::vector<std::string> readBoundingBoxLabels(const std::string& dataGroup);

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

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_ */
