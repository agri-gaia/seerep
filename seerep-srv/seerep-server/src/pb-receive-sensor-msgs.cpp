#include "seerep-server/pb-receive-sensor-msgs.h"

namespace seerep_server
{
PbReceiveSensorMsgs::PbReceiveSensorMsgs(std::shared_ptr<seerep_core::Core> seerepCore) : seerepCore(seerepCore)
{
}

grpc::Status PbReceiveSensorMsgs::TransferHeader(grpc::ServerContext* context, const seerep::Header* header,
                                                 seerep::ServerResponse* response)
{
  std::cout << "received header... " << std::endl;
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status PbReceiveSensorMsgs::TransferPoint(grpc::ServerContext* context, const seerep::Point* point,
                                                seerep::ServerResponse* response)
{
  std::cout << "received point... " << std::endl;
  // hdf5_io.writePoint("test_id", *point);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status PbReceiveSensorMsgs::TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion,
                                                     seerep::ServerResponse* response)
{
  std::cout << "received quaternion... " << std::endl;
  // hdf5_io.writeQuaternion("test_id", *quaternion);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status PbReceiveSensorMsgs::TransferPose(grpc::ServerContext* context, const seerep::Pose* pose,
                                               seerep::ServerResponse* response)
{
  std::cout << "received pose... " << std::endl;
  // hdf5_io.writePose("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status PbReceiveSensorMsgs::TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose,
                                                      seerep::ServerResponse* response)
{
  std::cout << "received pose_stamped... " << std::endl;
  // hdf5_io.writePoseStamped("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

} /* namespace seerep_server */
