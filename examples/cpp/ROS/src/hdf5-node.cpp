#include "seerep_ros_examples/hdf5-node.h"

namespace seerep_ros_examples
{
Hdf5Node::Hdf5Node(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle)
  : nh_{ node_handle }, pnh_{ private_node_handle }
{
  std::string basePath;
  if (!pnh_.getParam("basePath", basePath))
  {
    throw std::runtime_error("No base path for hdf5 files specified");
  }

  hdf5_access_ = std::make_unique<seerep_hdf5_ros::Hdf5Ros>(basePath);

  std::vector<std::string> topics;
  if (!pnh_.getParam("topics", topics))
  {
    throw std::runtime_error("No topics specified to dump");
  }

  for (std::string topic : topics)
  {
    ROS_INFO_STREAM("Trying to subscribe to topic " << topic);
  }

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  for (ros::master::TopicInfo info : topic_info)
  {
    auto find_iter = std::find(topics.begin(), topics.end(), info.name);
    if (find_iter != topics.end())
    {
      this->getSubscriber(info.datatype, info.name);
      topics.erase(find_iter);
    }
  }
}

void Hdf5Node::getSubscriber(const std::string& message_type, const std::string& topic)
{
  if (message_type == "sensor_msgs/Image")
  {
    image_subscriber_ = nh_.subscribe<sensor_msgs::Image>(topic, 0, &Hdf5Node::dumpMessage, this);
    ROS_INFO_STREAM("Subscribed to topic: " << topic << " with message type: " << message_type);
  }
  else
  {
    ROS_ERROR_STREAM("Type" << message_type << " not supported");
  }
}

void Hdf5Node::dumpMessage(const sensor_msgs::Image::ConstPtr& image) const
{
  hdf5_access_->dumpImage(*image);
  ROS_INFO("Callback for images called");
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
