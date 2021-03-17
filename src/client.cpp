#include "ag_grpc_ros/client.h"

// grpc
#include <grpc/grpc.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/security/credentials.h>

// ag
#include <ag_proto_msgs/SendPointCloud2.grpc.pb.h>
#include <ag_proto_ros/conversions.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace ag_grpc_ros
{

  class SendPointCloud2
  {

    public:
      SendPointCloud2(std::shared_ptr<grpc::Channel> channel_ptr)
        : stub_(ag::SendPointCloud2::NewStub(channel_ptr)) {}

      void sendTestCloud()
      {
        grpc::ClientContext context;
        sensor_msgs::PointCloud2 ros_cloud;
        ros_cloud.header.frame_id = "test_frame";
        ros_cloud.header.stamp = ros::Time::now();
        ag::PointCloud2 ag_cloud = ag_proto_ros::toProto(ros_cloud);
        ag::ServerResponse response;
        stub_->SendPointCloud2(&context, ag_cloud, &response);
        ROS_INFO_STREAM("Response:" << response.message());
      }

    private:
      std::unique_ptr<ag::SendPointCloud2::Stub> stub_;
  };


} /* namespace ag_grpc_client */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ag_grpc_ros_client");
  ros::NodeHandle private_nh("~");

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  ag_grpc_ros::SendPointCloud2 send_point_cloud_2(
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  send_point_cloud_2.sendTestCloud();

  return EXIT_SUCCESS;
}

