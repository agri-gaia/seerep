#ifndef SEEREP_HDF5_PB_HDF5_PB_IMAGE_H_
#define SEEREP_HDF5_PB_HDF5_PB_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5
#include <seerep_hdf5_core/hdf5_core_image.h>

#include "seerep-hdf5-pb/hdf5-pb-general.h"

// seerep-msgs
#include <seerep_msgs/image.pb.h>

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_pb
{
class Hdf5PbImage : public seerep_hdf5_core::Hdf5CoreImage, public Hdf5PbGeneral
{
public:
  Hdf5PbImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeImage(const std::string& id, const seerep::pb::Image& image);

  std::optional<seerep::pb::Image> readImage(const std::string& id);
};

}  // namespace seerep_hdf5_pb

#endif /* SEEREP_HDF5_PB_HDF5_PB_IMAGE_H_ */
