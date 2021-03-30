#include "ag_grpc_ros/client.h"
#include "ag_grpc_ros/types.h"
#include <grpc/status.h>

namespace ag_grpc_ros
{
TransferSensorMsgs::TransferSensorMsgs(std::shared_ptr<grpc::Channel> channel_ptr)
    : stub_(ag::TransferSensorMsgs::NewStub(channel_ptr)) {}


void ag_grpc_ros::TransferSensorMsgs::send(const std_msgs::Header::ConstPtr& msg) const
{
  grpc::ClientContext context;
  ag::ServerResponse response;
  grpc::Status status = stub_->TransferHeader(&context, ag_proto_ros::toProto(*msg), &response);
  if(!status.ok())
  {
    ROS_ERROR_STREAM("gGRPC status error code: " << status.error_code() << " " <<  status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void ag_grpc_ros::TransferSensorMsgs::send(const sensor_msgs::PointCloud2::ConstPtr& msg) const
{
  grpc::ClientContext context;
  ag::ServerResponse response;
  grpc::Status status = stub_->TransferPointCloud2(&context, ag_proto_ros::toProto(*msg), &response);
  if(!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " <<  status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void ag_grpc_ros::TransferSensorMsgs::send(const sensor_msgs::Image::ConstPtr& msg) const
{
  grpc::ClientContext context;
  ag::ServerResponse response;
  grpc::Status status = stub_->TransferImage(&context, ag_proto_ros::toProto(*msg), &response);
  if(!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " <<  status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

std::optional<ros::Subscriber> TransferSensorMsgs::getSubscriber(const std::string& message_type, const std::string& topic) {
  switch (ag_grpc_ros::type(message_type)) {
  case ag_grpc_ros::std_msgs_Header:
    return nh.subscribe<std_msgs::Header>(topic, 0, &TransferSensorMsgs::send, this);
  case ag_grpc_ros::sensor_msgs_Image:
    return nh.subscribe<sensor_msgs::Image>(topic, 0, &TransferSensorMsgs::send, this);
  case ag_grpc_ros::sensor_msgs_PointCloud2:
    return nh.subscribe<sensor_msgs::PointCloud2>(topic, 0, &TransferSensorMsgs::send, this);
  default:
    ROS_ERROR_STREAM("Type \"" << message_type << "\" not supported");
    return std::nullopt;
  }
}
} /* namespace ag_grpc_ros */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ag_grpc_ros_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::map<std::string, ros::Subscriber> subscribers;
  std::vector<std::string> topics;

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  ag_grpc_ros::TransferSensorMsgs transfer_sensor_msgs(
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  if(!private_nh.getParam("topics", topics))
  {
    ROS_WARN_STREAM("Use the \"topics\" parameter to specify the ROS topics which should be transferred! The \"topics\" parameter should be a list of strings.");
  }

  ROS_INFO_STREAM("Type names: " << ag_grpc_ros::names());


  for(auto topic : topics)
  {
    ROS_INFO_STREAM("Try to subscribe to topic \"" << topic << "\".");
  }

  for(auto info : topic_info)
  {
    auto find_iter = std::find(topics.begin(), topics.end(), info.name);

    if(find_iter != topics.end())
    {
      auto sub_opt = transfer_sensor_msgs.getSubscriber(info.datatype, info.name);
      if (sub_opt) {
        ROS_INFO_STREAM("Subscribe to topic: \"" << info.name << "\" of type:\"" << info.datatype << "\".");
        subscribers[info.name] = *sub_opt;
      }
      topics.erase(find_iter);
    }
    else
    {
      ROS_INFO_STREAM("Available Topics: \"" << info.name << "\"" );
    }
  }

  ros::spin();

  return EXIT_SUCCESS;
}


