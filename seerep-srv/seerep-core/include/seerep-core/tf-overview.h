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

// ros tf2
#include <tf2/buffer_core.h>
#include <ros/time.h>

namespace seerep_core
{
class TFOverview
{
public:
  TFOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io);
  ~TFOverview();
  std::vector<std::optional<seerep::TransformStamped>> getData(int64_t timesecs, int64_t timenanos,
                                                               std::string targetFrame, std::string sourceFrame);

  void addDataset(const seerep::TransformStamped& tf);

private:
  void recreateDatasets();
  void addToIndices(std::shared_ptr<seerep_core::TF> tf);
  void addToTfBuffer(seerep::TransformStamped transform);

  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;

  std::unordered_map<std::string, std::shared_ptr<seerep_core::TF>> m_datasets;

  tf2::BufferCore tfbuffer;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_TF_OVERVIEW_H_
