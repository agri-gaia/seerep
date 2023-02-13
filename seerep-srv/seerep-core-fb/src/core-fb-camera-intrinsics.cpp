#include "seerep-core-fb/core-fb-camera-intrinsics.h"

namespace seerep_core_fb
{
CoreFbCameraIntrinsics::CoreFbCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  CoreFbGeneral::getAllFileAccessorFromCore(m_seerepCore, m_hdf5IoMap);
}

CoreFbCameraIntrinsics::~CoreFbCameraIntrinsics()
{
}

// void getData(const seerep::fb::Query* query,
//              grpc::ServerWriter<flatbuffers::grpc::Message::<seerep::fb::CameraIntrinsics>>* const writer)
// {
//   BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
//       << "loading camera intrinsics from camera_intrinsics/";
// }

boost::uuids::uuid CoreFbCameraIntrinsics::addData(const seerep::fb::CameraIntrinsics& ci)
{
  seerep_core_msgs::camera_intrinsics ciCore;

  ciCore = seerep_core_fb::CoreFbConversion::fromFb(ci);
}
}  // namespace seerep_core_fb
