#include "seerep-core/core-camera-intrinsics.h"

namespace seerep_core
{
CoreTf::CoreTf(std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> hdf5_io) : m_hdf5_io(hdf5_io)
{
}
~CoreCameraIntrinsics()
{
}
void addData(seerep_core_msgs::camera_intrinsics& ci)
{
  m_hdf5_io->writeCameraIntrinsics(ci);
}
}  // namespace seerep_core
