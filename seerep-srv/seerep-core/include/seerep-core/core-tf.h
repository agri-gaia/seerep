#ifndef SEEREP_CORE_CORE_TF_H_
#define SEEREP_CORE_CORE_TF_H_

#include <functional>
#include <limits>
#include <optional>

// seerep-msgs
#include <seerep-msgs/aabb.h>
#include <seerep-msgs/query-tf.h>
#include <seerep-msgs/query.h>
// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// seerep-hdf5-core
#include <seerep-hdf5-core/hdf5-core-tf.h>

// ros tf2
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/buffer_core.h>
#include <tf2/transform_datatypes.h>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
namespace seerep_core
{
/**
 * @brief This is the class handling the TF buffer
 *
 * TFs are stored in a tf2::BufferCore. TFs can be added to this buffer and queries from one frame to another
 * can be queried for a specific point in time. AABBs and spatial queries can also be transformed directly in the
 * map frame of the project via public methods.
 */
class CoreTf
{
public:
  /**
   * @brief Constructs the object handling the TF buffer
   * @param hdf5_io pointer to the HDF5 io object for TFs
   */
  CoreTf(std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> hdf5_io);
  ~CoreTf();
  /**
   * @brief Returns the queried transformation between the two given frames at the given point in time
   * @param timesecs queried transformation time; seconds
   * @param timenanos queried transformation time; nanoseconds
   * @param targetFrame the target frame of the transformation
   * @param sourceFrame the source frame of the transformation
   * @return the queried TF if it exists
   */
  std::optional<geometry_msgs::TransformStamped> getData(const int64_t& timesecs, const int64_t& timenanos,
                                                         const std::string& targetFrame,
                                                         const std::string& sourceFrame);
  /**
   * @brief Adds a tf to the tf buffer
   * @param tf the TransformStamped to be added to the buffer
   */
  void addDataset(const geometry_msgs::TransformStamped& tf);

  /**
   * @brief Transforms an AABB into the target frame
   * @param aabb the AABB which should be transformed
   * @param sourceFrame the frame of the AABB before the transformation
   * @param targetFrame the frame of the AABB after the transformation
   * @param timeSecs the point in time to be used for the transformation; seconds
   * @param timeNanos the point in time to be used for the transformation; nanoseconds
   * @return the AABB transformed into the target frame
   */
  seerep_core_msgs::AABB transformAABB(seerep_core_msgs::AABB aabb, const std::string& sourceFrame,
                                       const std::string& targetFrame, const int64_t& timeSecs,
                                       const int64_t& timeNanos);
  /**
   * @brief Check if a transformation from the source to the target frame is available
   * at the defined point in time
   * @param sourceFrame the source frame of the transformation
   * @param targetFrame the target frame of the transformation
   * @param timeSecs the point in time to be used for the transformation; seconds
   * @param timeNanos the point in time to be used for the transformation; nanoseconds
   * @return true if transformation is available; false otherwise
   */
  bool canTransform(const std::string& sourceFrame, const std::string& targetFrame, const int64_t& timeSecs,
                    const int64_t& timeNanos);

  /**
   * @brief Transforms the spatial part of a query into the target frame
   * @param query the query which spatial part should be transformed
   * @param targetFrame the frame of the spatial part of the query after the transformation
   * @return the query with the spatial part transformed in the target frame
   */
  seerep_core_msgs::Query transformQuery(const seerep_core_msgs::Query& query, std::string targetFrame);
  /**
   * @brief Returns a vector of all frames stored in the TF tree by the TF buffer
   * @return vector of frame names
   */
  std::vector<std::string> getFrames();

private:
  /**
   * @brief loads the TFs into the buffer from the HDF5 file
   */
  void recreateDatasets();
  /**
   * @brief adds a transformation to the TF buffer
   * @param transform the transformation to be added to the buffer
   */
  void addToTfBuffer(geometry_msgs::TransformStamped transform);

  /** @brief shared pointer to the object handling the HDF5 io for TFs */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> m_hdf5_io;
  /** @brief the TF buffer */
  tf2::BufferCore m_tfBuffer;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_TF_H_
