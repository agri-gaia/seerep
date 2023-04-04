#include "seerep-core-fb/core-fb-camera-intrinsics.h"

namespace seerep_core_fb
{
CoreFbCameraIntrinsics::CoreFbCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  // CoreFbGeneral::getAllFileAccessorFromCore(m_seerepCore, m_hdf5IoMap);
}

CoreFbCameraIntrinsics::~CoreFbCameraIntrinsics()
{
}

void CoreFbCameraIntrinsics::getData(const seerep::fb::cameraIntrinsicsQuery& query)
// grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>>* const writer)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading camera intrinsics from camera_intrinsics/";

  seerep_core_msgs::camera_intrinsics_query ciq_coremsg;
  ciq_coremsg = seerep_core_fb::CoreFbConversion::fromFb(query);

  seerep_core_msgs::camera_intrinsics ci;
  m_seerepCore->getData(query, ci);
}

boost::uuids::uuid CoreFbCameraIntrinsics::setData(const seerep::fb::CameraIntrinsics& ci)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid projectuuid = gen(ci.header()->uuid_project()->str());

  seerep_core_msgs::camera_intrinsics ciCore;

  ciCore = seerep_core_fb::CoreFbConversion::fromFb(ci);

  m_seerepCore->addCameraIntrinsics(ci, projectuuid);
}
}  // namespace seerep_core_fb
