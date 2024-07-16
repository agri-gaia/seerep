#include "seerep_core/core_camera_intrinsics.h"

namespace seerep_core
{
CoreCameraIntrinsics::CoreCameraIntrinsics(
    std::shared_ptr<HighFive::File>& file,
    std::shared_ptr<std::mutex>& write_mtx)
{
  m_hdf5_io = std::make_shared<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>(
      file, write_mtx);
}
CoreCameraIntrinsics::~CoreCameraIntrinsics()
{
}
void CoreCameraIntrinsics::addData(
    const seerep_core_msgs::camera_intrinsics& cameraintrinsics)
{
  m_hdf5_io->writeCameraIntrinsics(cameraintrinsics);
}

std::optional<seerep_core_msgs::camera_intrinsics>
CoreCameraIntrinsics::getData(boost::uuids::uuid camIntrinsicsUuid)
{
  try
  {
    return m_hdf5_io->readCameraIntrinsics(camIntrinsicsUuid);
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << e.what();
    return std::nullopt;
  }
}

bool CoreCameraIntrinsics::cameraIntrinsicExists(
    boost::uuids::uuid camIntrinsicsUuid)
{
  return m_hdf5_io->cameraIntrinsicExists(camIntrinsicsUuid);
}

}  // namespace seerep_core
