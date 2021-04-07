#ifndef AG_GRPC_ROS_CLIENT_H_
#define AG_GRPC_ROS_CLIENT_H_

#include <functional>
#include <optional>

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

// pkg
#include "types.h"

namespace ag_grpc_ros
{
class TransferSensorMsgs
{
public:

  TransferSensorMsgs(std::shared_ptr<grpc::Channel> channel_ptr);

  std::optional<ros::Subscriber> getSubscriber(const std::string& message_type, const std::string& topic);

  void send(const std_msgs::Header::ConstPtr& msg) const;

  void send(const sensor_msgs::PointCloud2::ConstPtr& msg) const;

  void send(const sensor_msgs::Image::ConstPtr& msg) const;

private:
  StubPtr stub_;
  ros::NodeHandle nh;
};

} /* namespace ag_grpc_ros */

#endif // AG_GRPC_ROS_CLIENT_H_
