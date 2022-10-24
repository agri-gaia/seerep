#ifndef HDF5_NODE_H
#define HDF5_NODE_H

/* std */
#include <time.h>

#include <iostream>
#include <memory>
#include <optional>

/* ros */
#include "ros/master.h"
#include "ros/ros.h"

/* seerep-hdf5-ros */
#include "seerep-hdf5-ros/hdf5-ros.h"

namespace seerep_ros_examples
{
class Hdf5Node
{
public:
  Hdf5Node() = delete;
  Hdf5Node(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle);
  ~Hdf5Node() = default;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<seerep_hdf5_ros::Hdf5Ros> hdf5_access_;

  ros::Subscriber image_subscriber_;

  void getSubscriber(const std::string& message_type, const std::string& topic);

  void dumpMessage(const sensor_msgs::Image::ConstPtr& image) const;
};
} /* namespace seerep_ros_examples */

#endif /* HDF5_NODE_H_ */
