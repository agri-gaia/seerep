#include "seerep-core/core-camera-intrinsics.h"

namespace seerep_core
{
CoreCameraIntrinsics::CoreCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
}
CoreCameraIntrinsics::~CoreCameraIntrinsics()
{
}
void CoreCameraIntrinsics::addData(const seerep_core_msgs::camera_intrinsics& ci)
{
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> m_hdf5_io = getHdf5File(ci.header.uuidProject);
  m_hdf5_io->writeCameraIntrinsics(ci);
}

void CoreCameraIntrinsics::getData(seerep_core_msgs::camera_intrinsics_query& ci_query,
                                   seerep_core_msgs::camera_intrinsics& ci)
{
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> m_hdf5_io = getHdf5File(ci_query.uuidProject);
  ci = m_hdf5_io->readCameraIntrinsics(ci_query.uuidProject, ci_query.uuidCameraIntrinsics);
}

std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>
CoreCameraIntrinsics::getHdf5File(const boost::uuids::uuid& project_uuid)
{
  auto file = m_seerepCore->getHdf5File(project_uuid);
  auto mutex = m_seerepCore->getHdf5FileMutex(project_uuid);

  return std::make_shared<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>(file, mutex);
}

}  // namespace seerep_core
