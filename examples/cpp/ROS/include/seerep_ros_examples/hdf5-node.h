#ifndef HDF5_NODE_H
#define HDF5_NODE_H

// std
#include <chrono>
#include <optional>
#include <thread>

// ros
#include "ros/master.h"
#include "ros/ros.h"

// seerep
#include "seerep-hdf5-ros/hdf5-ros.h"

namespace seerep_ros_examples
{
class Hdf5Node
{
public:
  Hdf5Node() = delete;
  Hdf5Node(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle);

private:
  static constexpr unsigned int pollingIntervalInSeconds = 5;

  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;

  std::unique_ptr<seerep_hdf5_ros::Hdf5Ros> hdf5Access_;

  std::map<std::string, ros::Subscriber> subscribers_;

  bool topicAvailable(const std::string& topic, ros::master::TopicInfo& info);

  std::optional<ros::Subscriber> getSubscriber(const std::string& messageType, const std::string& topic);

  void dumpMessage(const sensor_msgs::Image::ConstPtr& image);
};
} /* namespace seerep_ros_examples */

#endif /* HDF5_NODE_H_ */
