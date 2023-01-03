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
/**
 * @brief ROS node to save ROS message directly into HDF5
 *
 * This Node saves supported ROS messages into HDF5, for that it try's to subscribe to the topics specified in the
 * launch file and saves the data with the seerep_hdf5_ros::Hdf5Ros class. Currently it's assumed that all the
 * topic's to subscribe to are available when this node is started.
 */
class Hdf5Node
{
public:
  Hdf5Node() = delete;
  /**
   * @brief Construct the ROS Node
   *
   * @param nh NodeHandle with a default namespace
   * @param pnh NodeHandle with a private namespace
   */
  Hdf5Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

private:
  /**
   * @brief Method to save ROS message to HDF5
   *
   * Delegates the call to the seerep_hdf5_ros::Hdf5Ros class.
   *
   * @tparam T The type of the message e.g sensor_msgs/Image
   * @param message The message to safe
   */
  template <typename T>
  void dumpMessage(const boost::shared_ptr<const T>& message);

  /**
   * @brief Get a parameter from the ROS parameter server
   *
   * Checks if the parameter is present in the parameter server and receives it's value. If the parameter is not
   * present, but it is required an exception is thrown.
   *
   * @tparam T The type of the parameter
   * @param parameterName The name of the parameter
   * @param parameterValue The value of the parameter
   * @param required Is the parameter required? Or can we handle it's absence.
   */
  template <typename T>
  void getROSParameter(const std::string& parameterName, T& parameterValue, bool required = true);

  /**
   * @brief Get a subscriber object for a topic
   *
   * Get a subscriber object with the respective callback to store the message.
   *
   * @param topic The name of the topic to subscribe to
   * @param messageType The message type of the topic, e.g sensor_msgs/Image
   * @return std::optional<ros::Subscriber> The subscriber if the message type is supported, otherwise empty
   */
  std::optional<ros::Subscriber> getSubscriber(const std::string& messageType, const std::string& topic);

  /**
   * @brief Get information about a specific topic
   *
   * @param topic_name The name of the topic
   * @return std::optional<ros::master::TopicInfo> The TopicInfo object if the topic is available, otherwise emtpy
   */
  std::optional<ros::master::TopicInfo> getTopicInfo(const std::string& topic_name);

  /**
   * @brief Checks wether the string is a valid Boost UUID
   *
   * @param uuid The string to check
   * @return true If the string is a valid UUID
   * @return false If the string is not a valid UUID
   */
  bool isValidUUID(const std::string& uuid) const;

  /**
   * @brief Stores topics, to which this node should subscribe to
   */
  std::vector<std::string> topics_;

  /**
   * @brief Store the subscriber objects in a map to keep them alive. The key is the topic name.
   */
  std::map<std::string, ros::Subscriber> subscribers_;

  /**
   * @brief Shared pointer to a write mutex for the HDf5 file
   *
   * Yes, a mutex wouldn't be necessary since only this process will write to the HDF5 file, but the
   * underlying storage classes currently require it.
   */
  std::shared_ptr<std::mutex> write_mutex_;

  /**
   * @brief Shared pointer to the Hdf5 file to write to
   */
  std::shared_ptr<HighFive::File> hdf5_file_;

  /**
   * @brief Unique Pointer to the seerep_hdf5_ros::Hdf5Ros class to delegate calls to
   */
  std::unique_ptr<seerep_hdf5_ros::Hdf5Ros> msg_dump_;

  /**
   * @brief NodeHandle in a default namespace
   */
  ros::NodeHandle node_handle_;

  /**
   * @brief NodeHandle with a private namespace
   */
  ros::NodeHandle private_node_handle_;
};
} /* namespace seerep_ros_examples */

#include "impl/hdf5-node.hpp"

#endif /* HDF5_NODE_H_ */
