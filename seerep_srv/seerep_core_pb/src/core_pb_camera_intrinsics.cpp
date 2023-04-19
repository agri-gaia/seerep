#include "seerep_core_pb/core_pb_camera_intrinsics.h"

namespace seerep_core_pb
{
// constructor when data received and stored to hdf5
CorePbCameraIntrinsics::CorePbCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
}

CorePbCameraIntrinsics::~CorePbCameraIntrinsics();
{
}

std::optional<seerep::pb::CameraIntrinsics> getData(const seerep::pb::CameraIntrinsicsQuery& query);
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading camera intrinsics from camera_intrinsics/";

  seerep_core_msgs::camera_intrinsics_query ciq_coremsg;
  // convert pb query to agnostic query
  ciq_coremsg = CorePbConversion::fromPb(query);

  std::optional<seerep_core_msgs::camera_intrinsics> cameraintrinsics;
  cameraintrinsics = m_seerepCore->getCameraIntrinsics(ciq_coremsg);

  seerep::pb::CameraIntrinsics camintrinsics_pb;
  // convert from agnostic cam intrinsics to pb cam intrinsics
  camintrinsics_pb = CorePbConversion::toPb(cameraintrinsics.value());
  return camintrinsics_pb;
}

boost::uuids::uuid addData(const seerep::pb::CameraIntrinsics& camintrinsics);
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid projectuuid = gen(cameraintrinsics.header()->uuid_project()->str());

  seerep_core_msgs::camera_intrinsics ciCore;
  // convert pb camintrinsics to agnostic camintrinsics
  ciCore = CorePbConversion::fromPb(camintrinsics);

  m_seerepCore->addCameraIntrinsics(ciCore, projectuuid);

  return projectuuid;
}

}  // namespace seerep_core_pb
