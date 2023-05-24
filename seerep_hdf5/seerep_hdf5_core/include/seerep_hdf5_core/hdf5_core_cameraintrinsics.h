#ifndef SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_

// highfive
#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>

// seerephdf5
#include "seerep_hdf5_core/hdf5_core_general.h"

// seerep-msgs
#include <header.h>
#include <region_of_interest.h>
#include <seerep_msgs/camera_intrinsics.h>

namespace seerep_hdf5_core
{
class Hdf5CoreCameraIntrinsics : public Hdf5CoreGeneral
{
public:
  Hdf5CoreCameraIntrinsics(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);
  /**
   * @brief Read camera intrinsics
   *
   * @param [in] cameraintrinsics_uuid boost uuid of a camera intrinsics object
   * @return seerep_core_msgs::camera_intrinsics [out] Retrieved camera intrinsics object
   */
  seerep_core_msgs::camera_intrinsics readCameraIntrinsics(const boost::uuids::uuid& cameraintrinsics_uuid);
  /**
   * @brief Write camera intrinics
   *
   * @param camIntrinsics [in] seerep core camera intrinsics object
   */
  void writeCameraIntrinsics(const seerep_core_msgs::camera_intrinsics& camIntrinsics);

  /**
   * @brief Check if there is an existing Camera Intrinsics against the provided UUID
   *
   * @param [in] cameraintrinsics_uuid boost uuid of a camera intrinsics object
   * @return true If camera intrinsics exist
   * @return false If camera intrinsics do not exist
   */
  bool checkCameraIntrinsicsExists(const boost::uuids::uuid& cameraintrinsics_uuid);

private:
  const std::shared_ptr<HighFive::DataSet> generateDatasetPointer(const std::string& attribute);
  const std::string getHdf5GroupPath(const std::string& id) const;
  const std::string getHdf5DataSetPath(const std::string& id) const;

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

  inline static const std::string REGION_OF_INTEREST_X_OFFSET = "region_of_interest/x_offset";
  inline static const std::string REGION_OF_INTEREST_Y_OFFSET = "region_of_interest/y_offset";
  inline static const std::string REGION_OF_INTEREST_HEIGHT = "region_of_interest/height";
  inline static const std::string REGION_OF_INTEREST_WIDTH = "region_of_interest/width";
  inline static const std::string REGION_OF_INTEREST_DO_RECTIFY = "region_of_interest/do_rectify";
};

}  // namespace seerep_hdf5_core

#endif  // SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_
