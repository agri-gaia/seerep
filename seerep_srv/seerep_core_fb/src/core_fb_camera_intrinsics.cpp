#include "seerep_core_fb/core_fb_camera_intrinsics.h"

namespace seerep_core_fb
{
CoreFbCameraIntrinsics::CoreFbCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
}

CoreFbCameraIntrinsics::~CoreFbCameraIntrinsics()
{
}

void CoreFbCameraIntrinsics::getData(
    const seerep::fb::CameraIntrinsicsQuery& query,
    grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>>* const writer)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading camera intrinsics from camera_intrinsics/";

  seerep_core_msgs::camera_intrinsics_query ciq_coremsg;
  ciq_coremsg = seerep_core_fb::CoreFbConversion::fromFb(query);

  std::optional<seerep_core_msgs::camera_intrinsics> cameraintrinsics;
  cameraintrinsics = m_seerepCore->getCameraIntrinsics(ciq_coremsg);

  flatbuffers::grpc::MessageBuilder mb;

  flatbuffers::Offset<seerep::fb::CameraIntrinsics> ci_fb;
  ci_fb = seerep_core_fb::CoreFbConversion::toFb(mb, cameraintrinsics.value());

  mb.Finish(ci_fb);
  auto msg = mb.ReleaseMessage<seerep::fb::CameraIntrinsics>();

  if (cameraintrinsics)
  {
    writer->Write(msg);
  }
  else
  {
    return;
  }
}

boost::uuids::uuid CoreFbCameraIntrinsics::setData(const seerep::fb::CameraIntrinsics& cameraintrinsics)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid projectuuid = gen(cameraintrinsics.header()->uuid_project()->str());

  seerep_core_msgs::camera_intrinsics ciCore;

  ciCore = seerep_core_fb::CoreFbConversion::fromFb(cameraintrinsics);

  m_seerepCore->addCameraIntrinsics(ciCore, projectuuid);

  return projectuuid;
}
}  // namespace seerep_core_fb
