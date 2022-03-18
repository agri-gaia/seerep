#ifndef SEEREP_HDF5_FB_IMAGE_H_
#define SEEREP_HDF5_FB_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include "seerep-hdf5-fb/hdf5-fb-general.h"

// seerep-msgs
#include <seerep-msgs/image_generated.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_hdf5_fb
{
class Hdf5FbImage : public Hdf5FbGeneral
{
public:
  Hdf5FbImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeImage(const std::string& id, const seerep::fb::Image& image);

  std::optional<flatbuffers::Offset<seerep::fb::Image>> readImage(const std::string& id, const std::string& projectuuid);

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

}  // namespace seerep_hdf5_fb

#endif /* SEEREP_HDF5_FB_IMAGE_H_ */
