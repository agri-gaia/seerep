#include "seerep_core/core_camera_intrinsics.h"

namespace seerep_core
{
CoreCameraIntrinsics::CoreCameraIntrinsics(std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> hdf5_io)
  : m_hdf5_io(hdf5_io)
{
}
CoreCameraIntrinsics::~CoreCameraIntrinsics()
{
}
void CoreCameraIntrinsics::addData(const seerep_core_msgs::camera_intrinsics& cameraintrinsics)
{
  m_hdf5_io->writeCameraIntrinsics(cameraintrinsics);
}

std::optional<seerep_core_msgs::camera_intrinsics> CoreCameraIntrinsics::getData(boost::uuids::uuid camIntrinsicsUuid)
{
  try
  {
    return m_hdf5_io->readCameraIntrinsics(camIntrinsicsUuid);
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << e.what();
    return std::nullopt;
  }
}

}  // namespace seerep_core
