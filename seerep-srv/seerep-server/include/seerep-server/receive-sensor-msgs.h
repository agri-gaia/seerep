#ifndef SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_
#define SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// seerep
#include <seerep-com/transfer-sensor-msgs.grpc.pb.h>
#include <seerep-hdf5/io.h>

#include <seerep-core/project-overview.h>

// this class is for all the gRPC calls that are not yet implemented in the server

namespace seerep_server
{
class ReceiveSensorMsgs final : public seerep::TransferSensorMsgs::Service
{
public:
  ReceiveSensorMsgs(std::shared_ptr<seerep_core::ProjectOverview> projectOverview);
  grpc::Status TransferHeader(grpc::ServerContext* context, const seerep::Header* header,
                              seerep::ServerResponse* response);

  grpc::Status TransferPoint(grpc::ServerContext* context, const seerep::Point* point, seerep::ServerResponse* response);

  grpc::Status TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion,
                                  seerep::ServerResponse* response);

  grpc::Status TransferPose(grpc::ServerContext* context, const seerep::Pose* pose, seerep::ServerResponse* response);

  grpc::Status TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose,
                                   seerep::ServerResponse* response);

private:
  std::shared_ptr<seerep_core::ProjectOverview> projectOverview;
};
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_
