#ifndef SEEREP_HDF5_ROS_H_
#define SEEREP_HDF5_ROS_H_

// Std
#include <mutex>
#include <string>

// HighFive
#include <highfive/H5File.hpp>

// Boost
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

// ROS
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Header.h"
#include "tf2_msgs/TFMessage.h"

// Seerep
#include "seerep-hdf5-core/hdf5-core-general.h"
#include "seerep-hdf5-core/hdf5-core-point-cloud.h"
#include "seerep-hdf5-core/hdf5-core-tf.h"

namespace seerep_hdf5_ros
{
/**
 * @brief Class for saving ROS messages into HDF5
 *
 * This class provides a HDF5 interface to save ROS messages. As a prerequisite, a HDF5 file with write
 * permissions and a mutex for thread safety are required. An example usage can be found in the
 * hdf5-ros node.
 *
 * @note Currently supports sensor_msgs::Image, sensor_msgs::PointCloud2, tf2_msgs::TFMessage
 */
class Hdf5Ros : public seerep_hdf5_core::Hdf5CoreTf, public seerep_hdf5_core::Hdf5CorePointCloud
{
public:
  Hdf5Ros() = delete;

  /**
   * @brief Construct a new Hdf5Ros Object
   *
   * @param hdf5File the hdf5 file where the messages should be written to
   * @param mutex the mutex for thread safety
   * @param projectName add a name for the collection of dumped messages (e.g "position_day_month_year")
   * @param projectFrameId name of the local world coordinate frame (e.g. "map")
   */
  Hdf5Ros(std::shared_ptr<HighFive::File>& hdf5File, std::shared_ptr<std::mutex>& mutex, const std::string& projectName,
          const std::string& projectFrameId);
  /**
   * @brief Write a ROS Image message to HDF5
   *
   * @param image the ROS image message
   */
  void saveMessage(const sensor_msgs::Image& image);

  /**
   * @brief Write a ROS PointCloud2 message to HDF5
   *
   * @param pointcloud the ROS PointCloud2 message
   */
  void saveMessage(const sensor_msgs::PointCloud2& pointcloud2);

  /**
   * @brief Write a ROS TransformStamped message to HDF5
   *
   * @param transformation the ROS TransformStamped message
   */
  void saveMessage(const tf2_msgs::TFMessage& transformation);

private:
  /**
   * @brief Write a ROS Header message to a HDF5 group
   *
   * The attributes are written as hdf5 attributes to the group
   *
   * @param hdf5DataSetPath path to the data group
   * @param header the ROS Header message
   */

  void saveHeader(const std::string& hdf5GroupPath, const std_msgs::Header& header);

  /** shared pointer to the currently used hdf5 file*/
  std::shared_ptr<HighFive::File> hdf5File_;
};
}  // namespace seerep_hdf5_ros
#endif /* SEEREP_HDF5_ROS_H_ */
