#include "ag_grpc_ros/server.h"

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// ag
#include <ag_proto_msgs/SendPointCloud2.grpc.pb.h>
#include <ag_proto_ros/conversions.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace ag_grpc_ros
{
  class ReceivePointCloud2 final : public ag::SendPointCloud2::Service
  {
  public:
    ReceivePointCloud2() {}
    ~ReceivePointCloud2() {}

    grpc::Status SendPointCloud2(grpc::ServerContext* context, const ag::PointCloud2* point_cloud_2, ag::ServerResponse* response)
    {
      sensor_msgs::PointCloud2 cloud = ag_proto_ros::toROS(*point_cloud_2);
      ROS_INFO_STREAM("Incoming PointCloud2 message" << std::endl << cloud);
      response->set_message("okidoki");
      response->set_transmission_state(ag::ServerResponse::SUCCESS);
      return grpc::Status::OK;
    }
  };

  std::shared_ptr<grpc::Server> createServer(
      const std::string& server_address,
      ag_grpc_ros::ReceivePointCloud2* point_cloud_2_service)
  {
    grpc::ServerBuilder server_builder;
    server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    server_builder.RegisterService(point_cloud_2_service);
    return std::shared_ptr<grpc::Server>(server_builder.BuildAndStart());
  }


} /* namespace ag_grpc_ros */


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ag_grpc_ros_server");
  ros::NodeHandle private_nh("~");

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  ag_grpc_ros::ReceivePointCloud2 point_cloud_2_service;
  std::shared_ptr<grpc::Server> server = ag_grpc_ros::createServer(server_address, &point_cloud_2_service);

  ROS_INFO_STREAM("Server listening on " << server_address);

  ros::spin();
  server->Shutdown();

  return EXIT_SUCCESS;
}
