#include "ag_grpc_ros/server.h"

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// Agri-Gaia grpc services
#include <ag_proto_msgs/SendPointCloud2.grpc.pb.h>


// ros
#include <ros/ros.h>


namespace ag_grpc_ros
{
  class ReceivePointCloud2 final : public ag::SendPointCloud2::Service
  {
  public:
    ReceivePointCloud2() {}
    ~ReceivePointCloud2() {}

    grpc::Status SendPointCloud2(grpc::ServerContext* context, const ag::PointCloud2* request, ag::ServerResponse* response)
    {
      response->set_message("okidoki");
      response->set_transmission_state(ag::ServerResponse::SUCCESS);
      return grpc::Status::OK;
    }
  };


  void runServer(const std::string& server_address)
  {
    grpc::ServerBuilder server_builder;
    server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    ReceivePointCloud2 point_cloud_2_service;
    server_builder.RegisterService(&point_cloud_2_service);
    std::unique_ptr<grpc::Server> server(server_builder.BuildAndStart());
    ROS_INFO_STREAM("Server listening on " << server_address);
    server->Wait();
  }


} /* namespace ag_grpc_ros */


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ag_grpc_ros_server");
  ros::NodeHandle private_nh("~");

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");
  ag_grpc_ros::runServer(server_address);

  return EXIT_SUCCESS;
}
