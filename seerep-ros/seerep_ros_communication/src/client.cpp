#include "seerep_ros_communication/client.h"
#include "seerep_ros_communication/types.h"
#include <grpc/status.h>

namespace seerep_grpc_ros
{
TransferSensorMsgs::TransferSensorMsgs(std::shared_ptr<grpc::Channel> channel_ptr)
  : stub_(seerep::TransferSensorMsgs::NewStub(channel_ptr)), stubMeta_(seerep::MetaOperations::NewStub(channel_ptr))
{
}

void seerep_grpc_ros::TransferSensorMsgs::send(const std_msgs::Header::ConstPtr& msg) const
{
  grpc::ClientContext context;
  seerep::ServerResponse response;
  grpc::Status status = stub_->TransferHeader(&context, seerep_ros_conversions::toProto(*msg), &response);
  if (!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void seerep_grpc_ros::TransferSensorMsgs::send(const sensor_msgs::PointCloud2::ConstPtr& msg) const
{
  grpc::ClientContext context;
  seerep::ServerResponse response;
  grpc::Status status = stub_->TransferPointCloud2(&context, seerep_ros_conversions::toProto(*msg), &response);
  if (!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void seerep_grpc_ros::TransferSensorMsgs::send(const sensor_msgs::Image::ConstPtr& msg) const
{
  grpc::ClientContext context;
  seerep::ServerResponse response;
  grpc::Status status = stub_->TransferImage(&context, seerep_ros_conversions::toProto(*msg), &response);
  if (!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void seerep_grpc_ros::TransferSensorMsgs::send(const geometry_msgs::Point::ConstPtr& msg) const
{
  grpc::ClientContext context;
  seerep::ServerResponse response;
  grpc::Status status = stub_->TransferPoint(&context, seerep_ros_conversions::toProto(*msg), &response);
  if (!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void seerep_grpc_ros::TransferSensorMsgs::send(const geometry_msgs::Quaternion::ConstPtr& msg) const
{
  grpc::ClientContext context;
  seerep::ServerResponse response;
  grpc::Status status = stub_->TransferQuaternion(&context, seerep_ros_conversions::toProto(*msg), &response);
  if (!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void seerep_grpc_ros::TransferSensorMsgs::send(const geometry_msgs::Pose::ConstPtr& msg) const
{
  grpc::ClientContext context;
  seerep::ServerResponse response;
  grpc::Status status = stub_->TransferPose(&context, seerep_ros_conversions::toProto(*msg), &response);
  if (!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void seerep_grpc_ros::TransferSensorMsgs::send(const geometry_msgs::PoseStamped::ConstPtr& msg) const
{
  grpc::ClientContext context;
  seerep::ServerResponse response;
  grpc::Status status = stub_->TransferPoseStamped(&context, seerep_ros_conversions::toProto(*msg), &response);
  if (!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.message());
  }
}

void seerep_grpc_ros::TransferSensorMsgs::send(const tf2_msgs::TFMessage::ConstPtr& msg) const
{
  grpc::ClientContext context;
  seerep::ServerResponse response;
  for (auto tf : msg->transforms)
  {
    grpc::Status status =
        stub_->TransferTransformStamped(&context, seerep_ros_conversions::toProto(tf, projectuuid), &response);
    if (!status.ok())
    {
      ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
    }
    else
    {
      ROS_INFO_STREAM("Response:" << response.message());
    }
  }
}

std::optional<ros::Subscriber> TransferSensorMsgs::getSubscriber(const std::string& message_type,
                                                                 const std::string& topic)
{
  switch (seerep_grpc_ros::type(message_type))
  {
    case seerep_grpc_ros::std_msgs_Header:
      return nh.subscribe<std_msgs::Header>(topic, 0, &TransferSensorMsgs::send, this);
    case seerep_grpc_ros::sensor_msgs_Image:
      return nh.subscribe<sensor_msgs::Image>(topic, 0, &TransferSensorMsgs::send, this);
    case seerep_grpc_ros::sensor_msgs_PointCloud2:
      return nh.subscribe<sensor_msgs::PointCloud2>(topic, 0, &TransferSensorMsgs::send, this);
    case seerep_grpc_ros::geometry_msgs_Point:
      return nh.subscribe<geometry_msgs::Point>(topic, 0, &TransferSensorMsgs::send, this);
    case seerep_grpc_ros::geometry_msgs_Quaternion:
      return nh.subscribe<geometry_msgs::Quaternion>(topic, 0, &TransferSensorMsgs::send, this);
    case seerep_grpc_ros::geometry_msgs_Pose:
      return nh.subscribe<geometry_msgs::Pose>(topic, 0, &TransferSensorMsgs::send, this);
    case seerep_grpc_ros::geometry_msgs_PoseStamped:
      return nh.subscribe<geometry_msgs::PoseStamped>(topic, 0, &TransferSensorMsgs::send, this);
    case tf2_msgs_TFMessage:
      return nh.subscribe<tf2_msgs::TFMessage>(topic, 0, &TransferSensorMsgs::send, this);
    default:
      ROS_ERROR_STREAM("Type \"" << message_type << "\" not supported");
      return std::nullopt;
  }
}

std::string TransferSensorMsgs::createProject(std::string projectname) const
{
  grpc::ClientContext context;
  seerep::ProjectInfo response;

  seerep::ProjectCreation projectcreation;
  *projectcreation.mutable_name() = projectname;

  grpc::Status status = stubMeta_->CreateProject(&context, projectcreation, &response);
  if (!status.ok())
  {
    ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
    return "";
  }
  else
  {
    ROS_INFO_STREAM("Response:" << response.name() << " " << response.uuid());
    return response.uuid();
  }
}

} /* namespace seerep_grpc_ros */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::map<std::string, ros::Subscriber> subscribers;
  std::vector<std::string> topics;

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  seerep_grpc_ros::TransferSensorMsgs transfer_sensor_msgs(
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  if (private_nh.getParam("projectUuid", transfer_sensor_msgs.projectuuid))
  {
    try
    {
      boost::uuids::string_generator gen;
      // if this throws no exception, the UUID is valid
      gen(transfer_sensor_msgs.projectuuid);
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string"
      std::cout << e.what() << std::endl;

      transfer_sensor_msgs.projectuuid = transfer_sensor_msgs.createProject("");
      ROS_WARN_STREAM("The provided UUID is invalid! Generating a a new one. (" + transfer_sensor_msgs.projectuuid +
                      ".h5)");
    }
  }
  else
  {
    transfer_sensor_msgs.projectuuid = transfer_sensor_msgs.createProject("");
    ROS_WARN_STREAM("Use the \"hdf5FolderPath\" parameter to specify the HDF5 file! Generating a a new one. (" +
                    transfer_sensor_msgs.projectuuid + ".h5)");
  }

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  if (!private_nh.getParam("topics", topics))
  {
    ROS_WARN_STREAM("Use the \"topics\" parameter to specify the ROS topics which should be transferred! The "
                    "\"topics\" parameter should be a list of strings.");
  }

  ROS_INFO_STREAM("Type names: " << seerep_grpc_ros::names());

  for (auto topic : topics)
  {
    ROS_INFO_STREAM("Try to subscribe to topic \"" << topic << "\".");
  }

  for (auto info : topic_info)
  {
    auto find_iter = std::find(topics.begin(), topics.end(), info.name);

    if (find_iter != topics.end())
    {
      auto sub_opt = transfer_sensor_msgs.getSubscriber(info.datatype, info.name);
      if (sub_opt)
      {
        ROS_INFO_STREAM("Subscribe to topic: \"" << info.name << "\" of type:\"" << info.datatype << "\".");
        subscribers[info.name] = *sub_opt;
      }
      topics.erase(find_iter);
    }
    else
    {
      ROS_INFO_STREAM("Available Topics: \"" << info.name << "\"");
    }
  }

  ros::spin();

  return EXIT_SUCCESS;
}
