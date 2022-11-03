#include "seerep_ros_examples/hdf5-node.h"

namespace seerep_ros_examples
{
Hdf5Node::Hdf5Node(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle)
  : nh_{ node_handle }, pnh_{ private_node_handle }
{
  std::string path;
  if (!pnh_.getParam("path", path))
  {
    throw std::runtime_error("No path specified");
  }

  std::string filename;
  if (!pnh_.getParam("filename", filename))
  {
    throw std::runtime_error("No filename for the hdf5 file");
  }

  hdf5_access_ = std::make_unique<seerep_hdf5_ros::Hdf5Ros>(path, filename);

  std::vector<std::string> topics;
  if (!pnh_.getParam("topics", topics))
  {
    throw std::runtime_error("No topics specified to dump");
  }

  for (std::string& topic : topics)
  {
    ros::master::TopicInfo topic_info;
    ROS_INFO_STREAM("Trying to subscribe to topic: " << topic);
    while (!topicAvailable(topic, topic_info))
    {
      ROS_INFO_STREAM("Waiting on topic: " << topic);
      std::this_thread::sleep_for(std::chrono::seconds(POLL_TIME));
    }
    std::optional<ros::Subscriber> sub = getSubscriber(topic_info.datatype, topic_info.name);
    if (sub)
    {
      subscribers_[topic_info.name] = *sub;
      ROS_INFO_STREAM("Subscribed to topic: " << topic_info.name << " of type: " << topic_info.datatype);
    }
  }
}

bool Hdf5Node::topicAvailable(const std::string& topic, ros::master::TopicInfo& info)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);
  auto find_iter = std::find_if(topic_info.begin(), topic_info.end(),
                                [&topic](const ros::master::TopicInfo& info) { return info.name == topic; });
  if (find_iter != topic_info.end())
  {
    info.name = find_iter->name;
    info.datatype = find_iter->datatype;
    return true;
  }
  return false;
}

std::optional<ros::Subscriber> Hdf5Node::getSubscriber(const std::string& message_type, const std::string& topic)
{
  if (message_type == "sensor_msgs/Image")
  {
    return nh_.subscribe<sensor_msgs::Image>(topic, 0, &Hdf5Node::dumpMessage, this);
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
  hdf5_access_->dumpImage(*image);
  ROS_INFO("Saved Image");
}

} /* namespace seerep_ros_examples */

int main(int argc, char** argv)
{
  const std::string& node_name = "hdf5-node";
  ros::init(argc, argv, node_name);

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  seerep_ros_examples::Hdf5Node node(nh, nh_private);

  ros::spin();
}
