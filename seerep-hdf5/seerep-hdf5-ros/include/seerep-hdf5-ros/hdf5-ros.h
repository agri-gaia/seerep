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
#include "std_msgs/Header.h"

// Seerep
#include "seerep-hdf5-core/hdf5-core-general.h"

namespace seerep_hdf5_ros
{
/**
 * @brief Class for saving ROS messages into HDF5
 *
 * This class enables to safe ROS messages directly into HDF5. As a prerequisite, a HDF5 file with write
 * permissions and a mutex for thread safety is required. The data field of a message is saved as a dataset,
 * additional message attributes are stored as attributes of the dataset.
 *
 * @note Currently only supports sensor_msgs::Image
 */
class Hdf5Ros : public seerep_hdf5_core::Hdf5CoreGeneral
{
public:
  Hdf5Ros() = delete;

  /**
   * @brief Construct a new Hdf5Ros Object, which handles ROS message saving
   *
   * @param hdf5File the hdf5 file the messages should be written to
   * @param mutex the mutex for thread safety
   * @param projectName add a name to the collection of dumped messages
   * @param projectFrameId name of the local world coordinate system (e.g. "map")
   */
  Hdf5Ros(std::shared_ptr<HighFive::File>& hdf5File, std::shared_ptr<std::mutex>& mutex, const std::string& projectName,
          const std::string& projectFrameId);

  /**
   * @brief Write ROS header message to HDF5
   *
   * The header attributes are written as attributes to a dataset
   *
   * @param hdf5DataSetPath path to the dataset, where the header should be added to
   * @param header the ROS header message
   */

  void saveHeader(const std::string& hdf5DataSetPath, const std_msgs::Header& header);

  /**
   * @brief Write ROS image message to HDF5
   *
   * The image data is written as a dataset, it's name (boost UUID) is generated automatically.
   *
   * @param image the ROS image message
   */
  void saveImage(const sensor_msgs::Image& image);
};
}  // namespace seerep_hdf5_ros
#endif /* SEEREP_HDF5_ROS_H_ */
