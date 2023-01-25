#ifndef SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include "seerep-hdf5-core/hdf5-core-datatype-interface.h"
#include "seerep-hdf5-core/hdf5-core-general.h"
#include "seerep-hdf5-core/hdf5-point-cloud2-iterator.h"

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

struct PCLIterInfos
{
  uint32_t height;
  uint32_t width;
  uint32_t pointStep;
  std::map<std::string, uint32_t> offsets;
  const uint8_t* data;
};

struct PCLChannels
{
  bool has_points = false;
  bool has_rgb = false;
  bool has_rgba = false;
  bool has_normals = false;
};

class Hdf5CorePointCloud : public virtual Hdf5CoreGeneral, public Hdf5CoreDatatypeInterface
{
public:
  Hdf5CorePointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const boost::uuids::uuid& uuid);
  std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const std::string& uuid);

  std::vector<std::string> getDatasetUuids();

  void writePCL(const std::string& pclUUID, PCLChannels channels, PCLIterInfos infos);
  std::vector<float> writePoints(const std::string& plcUUID, PCLIterInfos infos);
  void writeRGB(const std::string& plcUUID, PCLIterInfos infos);
  void writeRGBA(const std::string& plcUUID, PCLIterInfos infos);

  std::map<std::string, uint32_t> getOffsets(const std::vector<uint32_t>& offsets,
                                             const std::vector<std::string>& fieldNames, bool isBigendian);

  PCLChannels getChannels(const std::vector<std::string>& fields);

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
  inline static const std::string FIELD_NAME = "field_name";
  inline static const std::string FIELD_OFFSET = "field_offset";
  inline static const std::string FIELD_DATATYPE = "field_datatype";
  inline static const std::string FIELD_COUNT = "field_count";

  // make private again after fixing io calls of pointcloud.cpp and pointcloud-overview.cpp
  inline static const std::string BOUNDINGBOX = "boundingbox";
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_POINTCLOUD = "pointclouds";

private:
  uint32_t getRgbaOffset(const std::string& filename, uint32_t offset, bool isBigendian);
  void updateBoundingBox(std::array<float, 3>& min, std::array<float, 3>& max, const float& x, const float& y,
                         const float& z);
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_ */
