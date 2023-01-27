#include "seerep_ros_examples/hdf5-node.h"

namespace seerep_ros_examples
{
Hdf5Node::Hdf5Node(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle)
  : node_handle_{ nodeHandle }, private_node_handle_{ privateNodeHandle }
{
  std::string storage_path, filename, project_name, root_frame_id;

  try
  {
    getROSParameter<std::string>("storage_path", storage_path);
    getROSParameter<std::string>("project_name", project_name);
    getROSParameter<std::string>("project_root_frame", root_frame_id);
    getROSParameter<std::vector<std::string>>("topics", topics_);
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
    ros::shutdown();
  }

  // generate a random filename, to avoid appending to existing files
  filename = boost::lexical_cast<std::string>(boost::uuids::random_generator()()) + ".h5";
  ROS_INFO_STREAM("Saving to file: " << filename);

  write_mutex_ = std::make_shared<std::mutex>();
  hdf5_file_ = std::make_shared<HighFive::File>(storage_path + "/" + filename,
                                                HighFive::File::ReadWrite | HighFive::File::Create);

  msg_dump_ = std::make_unique<seerep_hdf5_ros::Hdf5Ros>(hdf5_file_, write_mutex_, project_name, root_frame_id);

  for (const std::string& topic : topics_)
  {
    ROS_INFO_STREAM("Trying to subscribe to topic: " << topic);

    std::optional<ros::master::TopicInfo> topic_info = getTopicInfo(topic);
    if (!topic_info)
    {
      ROS_ERROR_STREAM("Topic: " << topic << " not available");
      ros::shutdown();
    }

    std::optional<ros::Subscriber> subscriber = getSubscriber(topic_info.value().datatype, topic_info.value().name);
    if (subscriber)
    {
      subscribers_[topic_info.value().name] = *subscriber;
      ROS_INFO_STREAM("Subscribed to topic: " << topic << " of type: " << topic_info.value().datatype);
    }
    else
    {
      ROS_ERROR_STREAM("Topic: " << topic << " of type: " << topic_info.value().datatype << " not supported");
      ros::shutdown();
    }
  }
}

std::optional<ros::master::TopicInfo> Hdf5Node::getTopicInfo(const std::string& topic_name)
{
  std::vector<ros::master::TopicInfo> advertised_topics;
  ros::master::getTopics(advertised_topics);

  for (const auto& topic_info : advertised_topics)
  {
    if (topic_info.name == topic_name)
    {
      return topic_info;
    }
  }
  return std::nullopt;
}

std::optional<ros::Subscriber> Hdf5Node::getSubscriber(const std::string& message_type, const std::string& topic)
{
  if (message_type == "sensor_msgs/Image")
  {
    return node_handle_.subscribe<sensor_msgs::Image>(topic, 0, &Hdf5Node::dumpMessage<sensor_msgs::Image>, this);
  }
  else if (message_type == "sensor_msgs/PointCloud2")
  {
    return node_handle_.subscribe<sensor_msgs::PointCloud2>(topic, 0, &Hdf5Node::dumpMessage<sensor_msgs::PointCloud2>,
                                                            this);
  }
  else if (message_type == "tf2_msgs/TFMessage")
  {
    return node_handle_.subscribe<tf2_msgs::TFMessage>(topic, 0, &Hdf5Node::dumpMessage<tf2_msgs::TFMessage>, this);
  }
  return std::nullopt;
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
