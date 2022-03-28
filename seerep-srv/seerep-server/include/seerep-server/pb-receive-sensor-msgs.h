#ifndef SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_
#define SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

// seerep
#include <seerep-com/transfer-sensor-msgs.grpc.pb.h>
#include <seerep-core/core.h>

// this class is for all the gRPC calls that are not yet implemented in the server

namespace seerep_server
{
class PbReceiveSensorMsgs final : public seerep::TransferSensorMsgs::Service
{
public:
  PbReceiveSensorMsgs(std::shared_ptr<seerep_core::Core> seerepCore);
  grpc::Status TransferHeader(grpc::ServerContext* context, const seerep::Header* header,
                              seerep::ServerResponse* response);

  grpc::Status TransferPoint(grpc::ServerContext* context, const seerep::Point* point, seerep::ServerResponse* response);

  grpc::Status TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion,
                                  seerep::ServerResponse* response);

  grpc::Status TransferPose(grpc::ServerContext* context, const seerep::Pose* pose, seerep::ServerResponse* response);

  grpc::Status TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose,
                                   seerep::ServerResponse* response);

private:
  std::shared_ptr<seerep_core::Core> seerepCore;
};
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_
