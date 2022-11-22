#include "seerep_ros_communication/hdf5dump.h"

namespace seerep_grpc_ros
{
DumpSensorMsgs::DumpSensorMsgs(std::string hdf5FilePath)
{
  //m_labelsAsStdVector.push_back("testlabel_0");
  //m_labelsAsStdVector.push_back("testlabel_1");

  // no instances, just labels -> no uuids
  //m_instancesAsStdVector.push_back("");
  //m_instancesAsStdVector.push_back("");

  auto write_mtx = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> hdf5_file =
      std::make_shared<HighFive::File>(hdf5FilePath, HighFive::File::OpenOrCreate);
  m_ioTf = std::make_shared<seerep_hdf5_pb::Hdf5PbTf>(hdf5_file, write_mtx);
  m_ioPointCloud = std::make_shared<seerep_hdf5_pb::Hdf5PbPointCloud>(hdf5_file, write_mtx);
  m_ioImage = std::make_shared<seerep_hdf5_pb::Hdf5PbImage>(hdf5_file, write_mtx);
  m_ioImageCore = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(hdf5_file, write_mtx);
}

void DumpSensorMsgs::dump(const std_msgs::Header::ConstPtr& msg) const
{
  (void)msg;  // ignore that variable without causing warnings
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const sensor_msgs::PointCloud2::ConstPtr& msg) const
{
  std::string uuid = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
  ROS_INFO_STREAM("Dump point cloud 2 with uuid: " << uuid);
  try
  {
    m_ioPointCloud->writePointCloud2(uuid, seerep_ros_conversions_pb::toProto(*msg));
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception while saving point cloud: " << e.what());
  }
}

void DumpSensorMsgs::dump(const sensor_msgs::Image::ConstPtr& msg) const
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  std::string uuidString = boost::lexical_cast<std::string>(uuid);
  try
  {
    m_ioImage->writeImage(uuidString, seerep_ros_conversions_pb::toProto(*msg));

    // also write the labels general
    m_ioImageCore->writeLabelsGeneral(uuidString, m_labelsAsStdVector, m_instancesAsStdVector);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception while saving image: " << e.what());
  }
}

void DumpSensorMsgs::dump(const geometry_msgs::Point::ConstPtr& msg) const
{
  (void)msg;  // ignore that variable without causing warnings
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const geometry_msgs::Quaternion::ConstPtr& msg) const
{
  (void)msg;  // ignore that variable without causing warnings
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const geometry_msgs::Pose::ConstPtr& msg) const
{
  (void)msg;  // ignore that variable without causing warnings
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const geometry_msgs::PoseStamped::ConstPtr& msg) const
{
  (void)msg;  // ignore that variable without causing warnings
  ROS_INFO_STREAM("Datatype not implemented.");
}

void DumpSensorMsgs::dump(const tf2_msgs::TFMessage::ConstPtr& msg) const
{
  for (geometry_msgs::TransformStamped transform : msg->transforms)
  {
    try
    {
      m_ioTf->writeTransformStamped(seerep_ros_conversions_pb::toProto(transform));
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("Exception while saving transformation: " << e.what());
    }
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

// Add labels that will be written after image dump
void DumpSensorMsgs::addLabel(const std::string& key, const std::string& value){
  m_labelsAsStdVector.push_back(key);
  m_instancesAsStdVector.push_back(value);
}
}  // namespace seerep_grpc_ros

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_hdf5_dump");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::map<std::string, ros::Subscriber> subscribers;
  std::vector<std::string> topics;
  std::map<std::string, std::string> labels; // map with labels (key, value)

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
    catch (std::runtime_error const& e)
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

  if (!private_nh.getParam("labels", labels))
  {
    ROS_WARN_STREAM("Use the \"labels\" parameter to specify the labels which should be added! The "
                    "\"labels\" parameter should be key value pairs of strings, eg. weather: cloudy .");
  }

  for (auto const& label : labels)
  {
    ROS_INFO_STREAM("Add label \"" << label.first << ": " << label.second << "\".");
    dumpSensorMsgs.addLabel(label.first, label.second);
  }
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
