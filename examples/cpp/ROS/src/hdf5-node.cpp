#include "seerep_ros_examples/hdf5-node.h"

namespace seerep_ros_examples
{
Hdf5Node::Hdf5Node(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle)
  : nodeHandle_{ nodeHandle }, privateNodeHandle_{ privateNodeHandle }
{
  std::string path;
  if (!privateNodeHandle_.getParam("path", path))
  {
    throw std::runtime_error("No path specified");
  }

  // if a filename is not present or not a valid UUID, a proper UUID will be generated
  std::string filename;
  if (!privateNodeHandle_.getParam("filename", filename))
  {
    filename = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
    ROS_WARN_STREAM("No filename provided, using generated UUID instead");
  }

  if (!isValidUUID(filename))
  {
    filename = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
    ROS_WARN_STREAM("Filename is an invalid UUID, using generated UUID instead");
  }

  std::string projectName;
  if (!privateNodeHandle_.getParam("project_name", projectName))
  {
    throw std::runtime_error("No project name specified");
  }

  std::string rootFrameId;
  if (!privateNodeHandle_.getParam("project_root_frame", rootFrameId))
  {
    throw std::runtime_error("No root frame specified");
  }

  hdf5Access_ = std::make_unique<seerep_hdf5_ros::Hdf5Ros>(path, filename, rootFrameId, projectName);

  std::vector<std::string> topics;
  if (!privateNodeHandle_.getParam("topics", topics))
  {
    throw std::runtime_error("No topics specified to dump");
  }

  for (std::string& topic : topics)
  {
    ros::master::TopicInfo topicInfo;
    ROS_INFO_STREAM("Trying to subscribe to topic: " << topic);
    while (!topicAvailable(topic, topicInfo))
    {
      ROS_INFO_STREAM("Waiting on topic: " << topic);
      std::this_thread::sleep_for(std::chrono::seconds(pollingIntervalInSeconds));
    }
    std::optional<ros::Subscriber> sub = getSubscriber(topicInfo.datatype, topicInfo.name);
    if (sub)
    {
      subscribers_[topicInfo.name] = *sub;
      ROS_INFO_STREAM("Subscribed to topic: " << topicInfo.name << " of type: " << topicInfo.datatype);
    }
  }
}

bool Hdf5Node::isValidUUID(const std::string& uuid) const
{
  try
  {
    boost::uuids::uuid result = boost::uuids::string_generator()(uuid);
    return result.version() != boost::uuids::uuid::version_unknown;
  }
  catch (...)
  {
    return false;
  }
}

bool Hdf5Node::topicAvailable(const std::string& topic, ros::master::TopicInfo& info)
{
  ros::master::V_TopicInfo topicInfo;
  ros::master::getTopics(topicInfo);
  auto findIter = std::find_if(topicInfo.begin(), topicInfo.end(),
                               [&topic](const ros::master::TopicInfo& info) { return info.name == topic; });
  if (findIter != topicInfo.end())
  {
    info.name = findIter->name;
    info.datatype = findIter->datatype;
    return true;
  }
  return false;
}

std::optional<ros::Subscriber> Hdf5Node::getSubscriber(const std::string& message_type, const std::string& topic)
{
  if (message_type == "sensor_msgs/Image")
  {
    return nodeHandle_.subscribe<sensor_msgs::Image>(topic, 0, &Hdf5Node::dumpMessage, this);
    ROS_INFO_STREAM("Subscribed to topic: " << topic << " with message type: " << message_type);
  }
  else
  {
    ROS_ERROR_STREAM("Type" << message_type << " not supported");
    return std::nullopt;
  }
}

void Hdf5Node::dumpMessage(const sensor_msgs::Image::ConstPtr& image)
{
  hdf5Access_->dumpImage(*image);
  ROS_INFO("Saved Image");
}

} /* namespace seerep_ros_examples */

int main(int argc, char** argv)
{
  const std::string& node_name = "hdf5-node";
  ros::init(argc, argv, node_name);

  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateNodeHandle("~");

  seerep_ros_examples::Hdf5Node node(nodeHandle, privateNodeHandle);

  ros::spin();
}
