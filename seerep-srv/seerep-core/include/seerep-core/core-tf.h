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

namespace seerep_core
{
class CoreTf
{
public:
  CoreTf(std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> hdf5_io);
  ~CoreTf();
  std::optional<geometry_msgs::TransformStamped> getData(const int64_t& timesecs, const int64_t& timenanos,
                                                         const std::string& targetFrame,
                                                         const std::string& sourceFrame);

  void addDataset(const geometry_msgs::TransformStamped& tf);

  seerep_core_msgs::AABB transformAABB(seerep_core_msgs::AABB aabb, const std::string& sourceFrame,
                                       const std::string& targetFrame, const int64_t& timeSecs,
                                       const int64_t& timeNanos);

  bool canTransform(const std::string& sourceFrame, const std::string& targetFrame, const int64_t& timeSecs,
                    const int64_t& timeNanos);

  seerep_core_msgs::Query transformQuery(const seerep_core_msgs::Query& query, std::string targetFrame);

  std::vector<std::string> getFrames();

private:
  void recreateDatasets();
  void addToTfBuffer(geometry_msgs::TransformStamped transform);

  std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> m_hdf5_io;
  tf2::BufferCore m_tfbuffer;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_TF_H_
