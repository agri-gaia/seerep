#ifndef SEEREP_GRPC_ROS_CLIENT_H_
#define SEEREP_GRPC_ROS_CLIENT_H_

#include <functional>
#include <optional>

// grpc
#include <grpc/grpc.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/security/credentials.h>

// seerep
#include <seerep-com/transfer_sensor_msgs.grpc.pb.h>
#include <seerep-com/imageService.grpc.pb.h>
#include <seerep-com/pointCloudService.grpc.pb.h>
#include <seerep-com/tfService.grpc.pb.h>
#include <seerep_ros_conversions/conversions.h>

// ros
#include <ros/ros.h>
#include <ros/master.h>
#include <tf2_msgs/TFMessage.h>

// pkg
#include "types.h"

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_grpc_ros
{
class TransferSensorMsgs
{
public:
  TransferSensorMsgs(std::shared_ptr<grpc::Channel> channel_ptr);

  std::optional<ros::Subscriber> getSubscriber(const std::string& message_type, const std::string& topic);

  void send(const std_msgs::Header::ConstPtr& msg) const;

  void send(const sensor_msgs::PointCloud2::ConstPtr& msg) const;

  void send(const sensor_msgs::Image::ConstPtr& msg) const;

  void send(const geometry_msgs::Point::ConstPtr& msg) const;

  void send(const geometry_msgs::Quaternion::ConstPtr& msg) const;

  void send(const geometry_msgs::Pose::ConstPtr& msg) const;

  void send(const geometry_msgs::PoseStamped::ConstPtr& msg) const;

  void send(const tf2_msgs::TFMessage::ConstPtr& msg) const;

  std::string createProject(std::string projectname) const;

  std::string projectuuid;

private:
  StubTransferPtr stub_;
  StubImagePtr stubImage_;
  StubPointCloudPtr stubPointCloud_;
  StubTfPtr stubTf_;
  StubMetaPtr stubMeta_;
  ros::NodeHandle nh;
};

} /* namespace seerep_grpc_ros */

#endif  // SEEREP_GRPC_ROS_CLIENT_H_
