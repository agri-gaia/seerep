#ifndef SEEREP_CORE_CORE_TF_H_
#define SEEREP_CORE_CORE_TF_H_

#include <functional>
#include <optional>
#include <limits>

// seerep-msgs
#include <seerep-msgs/aabb.h>
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-tf.h>
// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// seerep-io-core
#include <seerep-io-core/io-core-tf.h>

// ros tf2
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <ros/time.h>

namespace seerep_core
{
class CoreTf
{
public:
  CoreTf(std::shared_ptr<seerep_io_core::IoCoreTf> hdf5_io);
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

  std::shared_ptr<seerep_io_core::IoCoreTf> m_hdf5_io;
  tf2::BufferCore m_tfbuffer;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_TF_H_
