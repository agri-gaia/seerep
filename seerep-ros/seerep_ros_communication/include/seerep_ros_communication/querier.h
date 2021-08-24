#ifndef SEEREP_GRPC_ROS_QUERIER_H_
#define SEEREP_GRPC_ROS_QUERIER_H_

#include <functional>
#include <optional>

// grpc
#include <grpc/grpc.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/security/credentials.h>

// ag
#include <seerep-com/transfer_sensor_msgs.grpc.pb.h>
#include <seerep_ros_conversions/conversions.h>

// ros
#include <ros/ros.h>
#include <ros/master.h>

// pkg
#include "types.h"

namespace seerep_grpc_ros
{
class QueryData
{
public:
  QueryData(std::shared_ptr<grpc::Channel> channel_ptr);

  void queryPointcloud(const seerep::Boundingbox& bb, ros::Publisher& pc2_pub) const;

private:
  StubPtr stub_;
  ros::NodeHandle nh;
};

} /* namespace seerep_grpc_ros */

#endif  // SEEREP_GRPC_ROS_QUERIER_H_
