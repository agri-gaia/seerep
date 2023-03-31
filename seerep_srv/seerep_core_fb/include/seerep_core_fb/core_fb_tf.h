#ifndef SEEREP_CORE_FB_TF_H_
#define SEEREP_CORE_FB_TF_H_

#include <functional>
#include <optional>

// seerep_core_fb
#include "core_fb_conversion.h"
#include "core_fb_general.h"

// seerep-msgs
#include <seerep_msgs/transform_stamped_generated.h>
#include <seerep_msgs/transform_stamped_query_generated.h>

// seerep_core_msgs
#include <seerep_msgs/query_tf.h>

// ros
#include <geometry_msgs/TransformStamped.h>

// seerep_hdf5_fb
#include <seerep_hdf5_fb/hdf5_fb_tf.h>

// seerep-conversion
#include <seerep_ros_conversions_fb/conversions.h>

// seerep-core
#include <seerep_core/core.h>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_core_fb
{
/**
 * @brief This class is the center piece between the gRPC interface, the core and the hdf5-io for tf
 */
class CoreFbTf
{
public:
  CoreFbTf(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbTf();

  /**
   * @brief Function to query tf
   * @param query the flatbuffer query for transformations
   * @param response the gRPC flatbuffer message containing the queried transformation
   */
  void getData(const seerep::fb::TransformStampedQuery& query,
               flatbuffers::grpc::Message<seerep::fb::TransformStamped>* response);
  /**
   * @brief Add a tf to the tf buffer and write it to hdf5
   * @param tf the flatbuffer message containing the tf
   *
   * The tf is stored in the hdf5 file via hdf5-io-fb. The tf is also added to the tf buffer.
   */
  void addData(const seerep::fb::TransformStamped& tf);

  /**
   * @brief gets all the frames which are in the tf tree of the project of interest
   * @param projectuuid the uuid of the project of interest
   * @return a vector of all the frames in the tf tree of the project
   *
   * The tf is stored in the hdf5 file via hdf5-io-fb. The tf is also added to the tf buffer.
   */
  std::vector<std::string> getFrames(const boost::uuids::uuid& projectuuid);

private:
  /** @brief a shared pointer to the general core */
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  /** a map from the uuids of the projects to the hdf5-io objects handling the io for the object */
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_fb::Hdf5FbTf>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
  /** the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_TF_H_
