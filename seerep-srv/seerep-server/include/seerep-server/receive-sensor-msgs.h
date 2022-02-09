#ifndef SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_
#define SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// seerep
#include <seerep-com/transfer_sensor_msgs.grpc.pb.h>
#include <seerep-hdf5/io.h>

#include <seerep-core/project-overview.h>

namespace seerep_server
{
class ReceiveSensorMsgs final : public seerep::TransferSensorMsgs::Service
{
public:
  ReceiveSensorMsgs(std::shared_ptr<seerep_core::ProjectOverview> projectOverview);
  grpc::Status TransferHeader(grpc::ServerContext* context, const seerep::Header* header,
                              seerep::ServerResponse* response);

  grpc::Status TransferImage(grpc::ServerContext* context, const seerep::Image* image, seerep::ServerResponse* response);

  grpc::Status TransferPointCloud2(grpc::ServerContext* context, const seerep::PointCloud2* pointCloud2,
                                   seerep::ServerResponse* response);

  grpc::Status TransferPoint(grpc::ServerContext* context, const seerep::Point* point, seerep::ServerResponse* response);

  grpc::Status TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion,
                                  seerep::ServerResponse* response);

  grpc::Status TransferPose(grpc::ServerContext* context, const seerep::Pose* pose, seerep::ServerResponse* response);

  grpc::Status TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose,
                                   seerep::ServerResponse* response);

  grpc::Status TransferTransformStamped(grpc::ServerContext* context, const seerep::TransformStamped* transform,
                                        seerep::ServerResponse* response);

private:
  std::shared_ptr<seerep_core::ProjectOverview> projectOverview;
};
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_RECEIVE_SENSOR_MSGS_H_
