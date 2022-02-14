#ifndef SEEREP_CORE_TF_OVERVIEW_H_
#define SEEREP_CORE_TF_OVERVIEW_H_

#include <functional>
#include <optional>
#include <limits>

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>
#include <seerep-msgs/query.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "tf.h"
#include "aabb-hierarchy.h"

// ros tf2
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <ros/time.h>

namespace seerep_core
{
class TFOverview
{
public:
  TFOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io);
  ~TFOverview();
  std::optional<seerep::TransformStamped> getData(const int64_t& timesecs, const int64_t& timenanos,
                                                  const std::string& targetFrame, const std::string& sourceFrame);

  void addDataset(const seerep::TransformStamped& tf);

  AabbHierarchy::AABB transformAABB(AabbHierarchy::AABB aabb, const std::string& sourceFrame,
                                    const std::string& targetFrame, const int64_t& timeSecs, const int64_t& timeNanos);

  bool canTransform(const std::string& sourceFrame, const std::string& targetFrame, const int64_t& timeSecs,
                    const int64_t& timeNanos);

  seerep::Query transformQuery(const seerep::Query& query, std::string targetFrame);

private:
  void recreateDatasets();
  void addToIndices(std::shared_ptr<seerep_core::TF> tf);
  void addToTfBuffer(seerep::TransformStamped transform);

  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;

  std::unordered_map<std::string, std::shared_ptr<seerep_core::TF>> m_datasets;

  tf2::BufferCore m_tfbuffer;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_TF_OVERVIEW_H_
