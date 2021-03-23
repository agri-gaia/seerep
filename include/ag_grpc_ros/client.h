#ifndef AG_GRPC_ROS_CLIENT_H_
#define AG_GRPC_ROS_CLIENT_H_

#include <functional>

// grpc
#include <grpc/grpc.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/security/credentials.h>

// ag
#include <ag_proto_msgs/transfer_sensor_msgs.grpc.pb.h>
#include <ag_proto_ros/conversions.h>

// ros
#include <ros/ros.h>
#include <ros/master.h>


namespace ag_grpc_ros
{

class TransferSensorMsgs
{
public:

  using StubPtr = std::unique_ptr<ag::TransferSensorMsgs::Stub>;

  TransferSensorMsgs(std::shared_ptr<grpc::Channel> channel_ptr);


 // template <typename Message>
  void send(const sensor_msgs::PointCloud2& ros_cloud)
  //void send(const Message& msg)
  {
    ag::PointCloud2 ag_cloud = ag_proto_ros::toProto(ros_cloud);
    grpc::ClientContext context;
    ag::ServerResponse response;
    stub_->TransferPointCloud2(&context, ag_proto_cloud, &response);
    ROS_INFO_STREAM("Response:" << response.message());
  }


  template <class T>
  struct grpcServiceFunctionBlub{
    const std::function<::grpc::Status(::grpc::ClientContext*, const ::ag::PointCloud2& request)> send_function = std::bind(&ag::TransferSensorMsgs::sendPointCloud2, stub_, _1, _2, _3);
  };

  template <>
  struct grpcServiceFunctionBlub<sensor_msgs::PointCloud2>{
    const std::function<::grpc::Status(::grpc::ClientContext*, const ::ag::PointCloud2& request)> send_function = std::bind(&ag::TransferSensorMsgs::sendPointCloud2, stub_, _1, _2, _3);
  };

private:
  StubPtr stub_;

};

} /* namespace ag_grpc_client */

#endif // AG_GRPC_ROS_CLIENT_H_
