#include "seerep-server/pb-receive-sensor-msgs.h"

namespace seerep_server
{
PbReceiveSensorMsgs::PbReceiveSensorMsgs(std::shared_ptr<seerep_core::Core> seerepCore) : seerepCore(seerepCore)
{
}

grpc::Status PbReceiveSensorMsgs::TransferHeader(grpc::ServerContext* context, const seerep::pb::Header* header,
                                                 seerep::pb::ServerResponse* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received header... ";
  response->set_message("okidoki");
  response->set_transmission_state(seerep::pb::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status PbReceiveSensorMsgs::TransferPoint(grpc::ServerContext* context, const seerep::pb::Point* point,
                                                seerep::pb::ServerResponse* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received point... ";
  // hdf5_io.writePoint("test_id", *point);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::pb::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status PbReceiveSensorMsgs::TransferQuaternion(grpc::ServerContext* context,
                                                     const seerep::pb::Quaternion* quaternion,
                                                     seerep::pb::ServerResponse* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received quaternion... ";
  // hdf5_io.writeQuaternion("test_id", *quaternion);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::pb::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status PbReceiveSensorMsgs::TransferPose(grpc::ServerContext* context, const seerep::pb::Pose* pose,
                                               seerep::pb::ServerResponse* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received pose... ";
  // hdf5_io.writePose("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::pb::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status PbReceiveSensorMsgs::TransferPoseStamped(grpc::ServerContext* context, const seerep::pb::PoseStamped* pose,
                                                      seerep::pb::ServerResponse* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received pose_stamped... ";
  // hdf5_io.writePoseStamped("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::pb::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

} /* namespace seerep_server */
