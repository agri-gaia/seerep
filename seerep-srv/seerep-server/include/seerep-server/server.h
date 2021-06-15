#ifndef SEEREP_SERVER_SERVER_H_
#define SEEREP_SERVER_SERVER_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// seerep
#include <seerep-com/transfer_sensor_msgs.grpc.pb.h>
#include <seerep-hdf5/io.h>

namespace seerep_server
{
class ReceiveSensorMsgs final : public seerep::TransferSensorMsgs::Service
{
public:
  ReceiveSensorMsgs(HighFive::File& file);
  grpc::Status TransferHeader(grpc::ServerContext* context, const seerep::Header* header,
                              seerep::ServerResponse* response);

  grpc::Status TransferImage(grpc::ServerContext* context, const seerep::Image* image, seerep::ServerResponse* response);

  grpc::Status TransferPointCloud2(grpc::ServerContext* context, const seerep::PointCloud2* point_cloud_2,
                                   seerep::ServerResponse* response);

  grpc::Status TransferPoint(grpc::ServerContext* context, const seerep::Point* point, seerep::ServerResponse* response);

  grpc::Status TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion,
                                  seerep::ServerResponse* response);

  grpc::Status TransferPose(grpc::ServerContext* context, const seerep::Pose* pose, seerep::ServerResponse* response);

  grpc::Status TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose,
                                   seerep::ServerResponse* response);

private:
  seerep_hdf5::SeerepHDF5IO hdf5_io;
};

std::shared_ptr<grpc::Server> createServer(const std::string& server_address,
                                           seerep_server::ReceiveSensorMsgs* receive_sensor_msgs);
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_SERVER_H_
