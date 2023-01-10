#ifndef SEEREP_GRPC_ROS_QUERIER_H_
#define SEEREP_GRPC_ROS_QUERIER_H_

#include <functional>
#include <optional>

// grpc
#include <grpc/grpc.h>
#include <grpc/status.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

// seerep
#include <seerep-com/image-service.grpc.pb.h>
#include <seerep-com/point-cloud-service.grpc.pb.h>
#include <seerep_ros_conversions_pb/conversions.h>

// ros
#include <ros/master.h>
#include <ros/ros.h>

// pkg
#include "types.h"

namespace seerep_grpc_ros
{
class QueryData
{
public:
  QueryData(std::shared_ptr<grpc::Channel> channel_ptr);

  void queryPointcloud(const seerep::pb::Query& query, ros::Publisher& pc2_pub) const;
  void queryImage(const seerep::pb::Query& query, ros::Publisher& img_pub) const;

private:
  StubImagePbPtr stubImage_;
  StubPointCloudPbPtr stubPointCloud_;
  ros::NodeHandle nh;
};

} /* namespace seerep_grpc_ros */

#endif  // SEEREP_GRPC_ROS_QUERIER_H_
