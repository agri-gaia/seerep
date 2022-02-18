#ifndef SEEREP_HDF5_IO_IMAGE_H_
#define SEEREP_HDF5_IO_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "ioGeneral.h"

// seerep-msgs
#include <seerep-msgs/image.pb.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_hdf5
{
class SeerepHDF5IOImage : public SeerepHDF5IOGeneral
{
public:
  SeerepHDF5IOImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeImage(const std::string& id, const seerep::Image& image);

  std::optional<seerep::Image> readImage(const std::string& id);

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

  std::shared_ptr<HighFive::File> m_file;
  std::shared_ptr<std::mutex> m_write_mtx;

public:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_IMAGE = "images";
};

} /* namespace seerep_hdf5 */

#endif /* SEEREP_HDF5_IO_IMAGE_H_ */
