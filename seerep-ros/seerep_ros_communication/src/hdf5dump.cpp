#include "seerep_ros_communication/hdf5dump.h"

namespace seerep_grpc_ros
{
DumpSensorMsgs::DumpSensorMsgs(std::string hdf5FilePath)
{
  HighFive::File hdf5_file(hdf5FilePath, HighFive::File::OpenOrCreate);
  m_hdf5_io = std::make_shared<seerep_hdf5::SeerepHDF5IO>(hdf5_file);
}

void DumpSensorMsgs::dump(const std_msgs::Header::ConstPtr& msg) const
{
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const sensor_msgs::PointCloud2::ConstPtr& msg) const
{
  std::string uuid = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
  ROS_INFO_STREAM("Dump point cloud 2 with uuid: " << uuid);
  m_hdf5_io->writePointCloud2(uuid, seerep_ros_conversions::toProto(*msg));
}

void DumpSensorMsgs::dump(const sensor_msgs::Image::ConstPtr& msg) const
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  m_hdf5_io->writeImage(boost::lexical_cast<std::string>(uuid), seerep_ros_conversions::toProto(*msg));
}

void DumpSensorMsgs::dump(const geometry_msgs::Point::ConstPtr& msg) const
{
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const geometry_msgs::Quaternion::ConstPtr& msg) const
{
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const geometry_msgs::Pose::ConstPtr& msg) const
{
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const geometry_msgs::PoseStamped::ConstPtr& msg) const
{
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const tf2_msgs::TFMessage::ConstPtr& msg) const
{
  for (geometry_msgs::TransformStamped transform : msg->transforms)
  {
    m_hdf5_io->writeTransformStamped(seerep_ros_conversions::toProto(transform));
  }
}

std::optional<ros::Subscriber> DumpSensorMsgs::getSubscriber(const std::string& message_type, const std::string& topic)
{
  switch (type(message_type))
  {
    case std_msgs_Header:
      return nh.subscribe<std_msgs::Header>(topic, 0, &DumpSensorMsgs::dump, this);
    case sensor_msgs_Image:
      return nh.subscribe<sensor_msgs::Image>(topic, 0, &DumpSensorMsgs::dump, this);
    case sensor_msgs_PointCloud2:
      return nh.subscribe<sensor_msgs::PointCloud2>(topic, 0, &DumpSensorMsgs::dump, this);
    case geometry_msgs_Point:
      return nh.subscribe<geometry_msgs::Point>(topic, 0, &DumpSensorMsgs::dump, this);
    case geometry_msgs_Quaternion:
      return nh.subscribe<geometry_msgs::Quaternion>(topic, 0, &DumpSensorMsgs::dump, this);
    case geometry_msgs_Pose:
      return nh.subscribe<geometry_msgs::Pose>(topic, 0, &DumpSensorMsgs::dump, this);
    case geometry_msgs_PoseStamped:
      return nh.subscribe<geometry_msgs::PoseStamped>(topic, 0, &DumpSensorMsgs::dump, this);
    case tf2_msgs_TFMessage:
      return nh.subscribe<tf2_msgs::TFMessage>(topic, 0, &DumpSensorMsgs::dump, this);
    default:
      ROS_ERROR_STREAM("Type \"" << message_type << "\" not supported");
      return std::nullopt;
  }
}
}  // namespace seerep_grpc_ros

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_hdf5_dump");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::map<std::string, ros::Subscriber> subscribers;
  std::vector<std::string> topics;

  std::string hdf5FolderPath;
  if (!private_nh.getParam("hdf5FolderPath", hdf5FolderPath))
  {
    ROS_WARN_STREAM("Use the \"hdf5FolderPath\" parameter to specify the HDF5 file!");
    return EXIT_FAILURE;
  }

  std::string projectUuid;
  if (private_nh.getParam("projectUuid", projectUuid))
  {
    try
    {
      boost::uuids::string_generator gen;
      // if this throws no exception, the UUID is valid
      gen(projectUuid);
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string"
      std::cout << e.what() << std::endl;
      projectUuid = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      ROS_WARN_STREAM("The provided UUID is invalid! Generating a a new one. (" + projectUuid + ".h5)");
    }
  }
  else
  {
    projectUuid = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
    ROS_WARN_STREAM("Use the \"hdf5FolderPath\" parameter to specify the HDF5 file! Generating a a new one. (" +
                    projectUuid + ".h5)");
  }

  std::string hdf5FilePath = hdf5FolderPath + "/" + projectUuid + ".h5";

  seerep_grpc_ros::DumpSensorMsgs dumpSensorMsgs = seerep_grpc_ros::DumpSensorMsgs(hdf5FilePath);

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
    ROS_INFO_STREAM("Trying to subscribe to topic \"" << topic << "\".");
  }

  for (auto info : topic_info)
  {
    auto find_iter = std::find(topics.begin(), topics.end(), info.name);

    if (find_iter != topics.end())
    {
      auto sub_opt = dumpSensorMsgs.getSubscriber(info.datatype, info.name);
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
