#include "seerep-core/core-camera-intrinsics.h"

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

std::optional<seerep_core_msgs::camera_intrinsics>
CoreCameraIntrinsics::getData(const seerep_core_msgs::camera_intrinsics_query& cameraintrinsics_query)
{
  try
  {
    return m_hdf5_io->readCameraIntrinsics(cameraintrinsics_query.uuidProject,
                                           cameraintrinsics_query.uuidCameraIntrinsics);
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << e.what();
    return std::nullopt;
  }
}

}  // namespace seerep_core
