#include "seerep-hdf5-core/hdf5-core-cameraintrinsics.h"

namespace seerep_hdf5_core
{
Hdf5CoreCameraIntrinsics::Hdf5CoreCameraIntrinsics(std::shared_ptr<HighFive::File>& file,
                                                   std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

void Hdf5CoreCameraIntrinsics::writeCameraIntrinsics(const seerep_core_msgs::camera_intrinsics& camIntrinsics)
{
  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DatasetPath = getHdf5DataSetPath(id);

  auto dataGroupPtr = getHdf5Group(hdf5GroupPath, false);
  auto dataSetPtr = getHdf5DataSet(hdf5DatasetPath);

  if (dataGroupPtr && dataSetPtr)
  {
    writeHeader(camIntrinsics->header);
    writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::HEIGHT,
                                   camIntrinsics->height);
    writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::WIDTH,
                                   camIntrinsics->width);
    writeAttributeToHdf5<std::string>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::DISTORTION_MODEL,
                                      camIntrinsics->distortion);
    writeAttributeToHdf5<std::vector<double>>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::DISTORTION,
                                              camIntrinsics->distortion);
    writeAttributeToHdf5<std::vector<double>>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::INTRINSIC_MATRIX, camIntrinsics->intrinsic_matrix);
    writeAttributeToHdf5<std::vector<double>>(*dataGroupPtr,
                                              seerep_hdf5_core::Hdf5CoreCameraIntrinsics::RECTIFICATION_MATRIX,
                                              camIntrinsics->rectification_matrix);
    writeAttributeToHdf5<std::vector<double>>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::PROJECTION_MATRIX, camIntrinsics->projection_matrix);
    writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::binning_x,
                                   camIntrinsics->binning_x);
    writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::binning_y,
                                   camIntrinsics->binning_y);
  }
}

}  // namespace seerep_hdf5_core
