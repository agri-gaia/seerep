#ifndef HDF5_NODE_H
#define HDF5_NODE_H

// std
#include <chrono>
#include <optional>
#include <thread>

// ros
#include "ros/master.h"
#include "ros/ros.h"

// boost
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_generators.hpp>

// seerep
#include "seerep-hdf5-ros/hdf5-ros.h"

namespace seerep_ros_examples
{
class Hdf5Node
{
public:
  Hdf5Node() = delete;
  Hdf5Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

private:
  template <typename T>
  void dumpMessage(const boost::shared_ptr<const T>& message);

  template <typename T>
  void getROSParameter(const std::string& parameterName, T& parameterValue, bool required = true);

  std::optional<ros::Subscriber> getSubscriber(const std::string& messageType, const std::string& topic);
  std::optional<ros::master::TopicInfo> getTopicInfo(const std::string& topic_name);

  bool isValidUUID(const std::string& uuid) const;

  std::vector<std::string> topics_;
  std::map<std::string, ros::Subscriber> subscribers_;

  std::shared_ptr<std::mutex> write_mutex_;
  std::shared_ptr<HighFive::File> hdf5_file_;
  std::unique_ptr<seerep_hdf5_ros::Hdf5Ros> msg_dump_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
};
} /* namespace seerep_ros_examples */

#include "impl/hdf5-node.hpp"

#endif /* HDF5_NODE_H_ */
