#ifndef SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_

// highfive
#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>

// seerephdf5
#include "seerep-hdf5-core/hdf5-core-general.h"

// seerep-msgs
#include <header.h>
#include <region_of_interest.h>
#include <seerep-msgs/camera_intrinsics.h>

namespace seerep_hdf5_core
{
class Hdf5CoreCameraIntrinsics : public Hdf5CoreGeneral
{
public:
  Hdf5CoreCameraIntrinsics(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);
  std::optional<std::vector<seerep_core_msgs::camera_intrinsics>> readCameraIntrinsics(const std::string& id);
  void writeCameraIntrinsics(const seerep_core_msgs::camera_intrinsics);

public:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_CAMINTRINSICS = "cameraintrinsics";
  inline static const std::string SIZE = "size";

  inline static const std::string HEIGHT = "height";
  inline static const std::string WIDTH = "width";
  inline static const std::string DISTORTION_MODEL = "distortion_model";
  inline static const std::string DISTORTION = "distortion";
  inline static const std::string INTRINSIC_MATRIX = "intrinsic_matrix";
  inline static const std::string RECTIFICATION_MATRIX = "rectification_matrix";
  inline static const std::string PROJECTION_MATRIX = "projection_matrix";
  inline static const std::string BINNING_X = "binning_x";
  inline static const std::string BINNING_Y = "binning_y";
  inline static const std::string REGION_OF_INTEREST = "region_of_interest";
};
}
}  // namespace seerep_hdf5_core

#endif  // SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_
