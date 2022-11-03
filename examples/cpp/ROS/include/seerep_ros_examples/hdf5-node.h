#ifndef HDF5_NODE_H
#define HDF5_NODE_H

// std
#include <chrono>
#include <memory>
#include <optional>
#include <thread>

// ros
#include "ros/master.h"
#include "ros/ros.h"

// seerep
#include "seerep-hdf5-ros/hdf5-ros.h"

#define POLL_TIME 5  // in seconds

namespace seerep_ros_examples
{
class Hdf5Node
{
public:
  Hdf5Node() = delete;
  Hdf5Node(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<seerep_hdf5_ros::Hdf5Ros> hdf5_access_;

  std::map<std::string, ros::Subscriber> subscribers_;

  bool topicAvailable(const std::string& topic, ros::master::TopicInfo& info);

  std::optional<ros::Subscriber> getSubscriber(const std::string& message_type, const std::string& topic);

  void dumpMessage(const sensor_msgs::Image::ConstPtr& image);
};
} /* namespace seerep_ros_examples */

#endif /* HDF5_NODE_H_ */
