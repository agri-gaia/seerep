#include "seerep_ros_communication/server.h"

namespace seerep_grpc_ros
{
ReceiveSensorMsgs::ReceiveSensorMsgs() {}

grpc::Status ReceiveSensorMsgs::TransferPointCloud2(
    grpc::ServerContext* context,
    const seerep::PointCloud2* msg,
    seerep::ServerResponse* response)
{
  sensor_msgs::PointCloud2 cloud = seerep_ros_conversions::toROS(*msg);
  ROS_INFO_STREAM("Incoming PointCloud2 message" << std::endl << cloud);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferHeader(
    grpc::ServerContext* context,
    const seerep::Header* msg,
    seerep::ServerResponse* response)
{
  std_msgs::Header header = seerep_ros_conversions::toROS(*msg);
  ROS_INFO_STREAM("Incoming Header message" << std::endl << header);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferImage(
    grpc::ServerContext* context,
    const seerep::Image* msg,
    seerep::ServerResponse* response)
{
  sensor_msgs::Image image = seerep_ros_conversions::toROS(*msg);
  ROS_INFO_STREAM("Incoming Image message" << std::endl << image);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPoint(
    grpc::ServerContext* context,
    const seerep::Point* msg,
    seerep::ServerResponse* response)
{
  geometry_msgs::Point point = seerep_ros_conversions::toROS(*msg);
  ROS_INFO_STREAM("Incoming Point message" << std::endl << point);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferQuaternion(
    grpc::ServerContext* context,
    const seerep::Quaternion* msg,
    seerep::ServerResponse* response)
{
  geometry_msgs::Quaternion quaternion = seerep_ros_conversions::toROS(*msg);
  ROS_INFO_STREAM("Incoming Quaternion message" << std::endl << quaternion);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPose(
    grpc::ServerContext* context,
    const seerep::Pose* msg,
    seerep::ServerResponse* response)
{
  geometry_msgs::Pose pose = seerep_ros_conversions::toROS(*msg);
  ROS_INFO_STREAM("Incoming Pose message" << std::endl << pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPoseStamped(
    grpc::ServerContext* context,
    const seerep::PoseStamped* msg,
    seerep::ServerResponse* response)
{
  geometry_msgs::PoseStamped pose = seerep_ros_conversions::toROS(*msg);
  ROS_INFO_STREAM("Incoming PoseStamped message" << std::endl << pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

std::shared_ptr<grpc::Server> createServer(
    const std::string& server_address,
    seerep_grpc_ros::ReceiveSensorMsgs* receive_sensor_msgs)
{
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(receive_sensor_msgs);
  return std::shared_ptr<grpc::Server>(server_builder.BuildAndStart());
}

} /* namespace seerep_grpc_ros */


int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_server");
  ros::NodeHandle private_nh("~");

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  seerep_grpc_ros::ReceiveSensorMsgs receive_sensor_msgs_service;
  std::shared_ptr<grpc::Server> server = seerep_grpc_ros::createServer(server_address, &receive_sensor_msgs_service);

  ROS_INFO_STREAM("Server listening on " << server_address);

  ros::spin();
  server->Shutdown();

  return EXIT_SUCCESS;
}
