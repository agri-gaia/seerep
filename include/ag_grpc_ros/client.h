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

// pkg
#include "types.h"

namespace ag_grpc_ros
{
class TransferSensorMsgs
{
public:


  TransferSensorMsgs(std::shared_ptr<grpc::Channel> channel_ptr);

  void send(const sensor_msgs::PointCloud2& ros_cloud);

private:
  StubPtr stub_;

};

} /* namespace ag_grpc_client */

#endif // AG_GRPC_ROS_CLIENT_H_
