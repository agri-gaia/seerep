#ifndef AG_GRPC_HDF5_SERVER_H_
#define AG_GRPC_HDF5_SERVER_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// ag
#include <ag_proto_msgs/transfer_sensor_msgs.grpc.pb.h>
#include <ag_proto_hdf5/io.h>

namespace ag_grpc_hdf5 {
class ReceiveSensorMsgs final : public ag::TransferSensorMsgs::Service {
public:
  ReceiveSensorMsgs(HighFive::File &file);
  grpc::Status transferPointCloud2(grpc::ServerContext *context,
                                   const ag::PointCloud2 *point_cloud_2,
                                   ag::ServerResponse *response);

private:
  ag_proto_hdf5::AGProtoHDF5IO hdf5_io;
};

std::shared_ptr<grpc::Server>
createServer(const std::string &server_address,
             ag_grpc_hdf5::ReceiveSensorMsgs *receive_sensor_msgs);
}
#endif // AG_GRPC_HDF5_SERVER_H_
