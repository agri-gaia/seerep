#ifndef SEEREP_HDF5_IO_IMAGE_FBS_H_
#define SEEREP_HDF5_IO_IMAGE_FBS_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "seerep-fb-io/general-io-fbs.h"

// seerep-msgs
#include <seerep-msgs/image_generated.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_fb_io
{
class ImageIOFbs : public GeneralIOFbs
{
public:
  ImageIOFbs(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeImage(const std::string& id, const seerep::fb::Image& image);

  std::optional<flatbuffers::Offset<seerep::fb::Image>> readImage(const std::string& id);

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

} /* namespace seerep_fb_io */

#endif /* SEEREP_HDF5_IO_IMAGE_FBS_H_ */
