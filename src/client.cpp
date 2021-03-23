#include "ag_grpc_ros/client.h"
#include "ag_grpc_ros/types.h"

namespace ag_grpc_ros
{
TransferSensorMsgs::TransferSensorMsgs(std::shared_ptr<grpc::Channel> channel_ptr)
    : stub_(ag::TransferSensorMsgs::NewStub(channel_ptr)) {}


void ag_grpc_ros::TransferSensorMsgs::send(const sensor_msgs::PointCloud2& ros_cloud)
{
  ag::PointCloud2 ag_cloud = ag_proto_ros::toProto(ros_cloud);
  grpc::ClientContext context;
  ag::ServerResponse response;
  stub_->TransferPointCloud2(&context, ag_cloud, &response);
  ROS_INFO_STREAM("Response:" << response.message());
}




} /* namespace ag_grpc_client */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ag_grpc_ros_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::map<std::string, ros::Subscriber> subscribers;
  std::vector<std::string> topics;
  nh.getParam("topics", topics);

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  ROS_INFO_STREAM("Type names: " << ag_grpc_ros::names());

  for(auto info : topic_info)
  {
    ROS_INFO_STREAM("Topic: " << info.name << " type:" << info.datatype);
    switch (ag_grpc_ros::type(info.datatype)) {
    case ag_grpc_ros::sensor_msgs_PointCloud2:
        ROS_INFO_STREAM(info.datatype);
        break;
    case ag_grpc_ros::std_msgs_Header:
        ROS_INFO_STREAM(info.datatype);
        break;
      default:
        ROS_ERROR_STREAM("Type \"" << info.datatype << "\" of topic \"" << info.name << "\" not supported");
        break;
    }
  }

  for(auto topic : topics)
  {
    //topic_tools::ShapeShifter shifter;
    //shifter.advertise(nh, topic, 0, )
  }

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");


  ag_grpc_ros::TransferSensorMsgs transfer_sensor_msgs(
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  ros::spin();

  return EXIT_SUCCESS;
}


