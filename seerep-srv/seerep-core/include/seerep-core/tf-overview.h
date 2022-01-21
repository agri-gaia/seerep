#ifndef SEEREP_CORE_TF_OVERVIEW_H_
#define SEEREP_CORE_TF_OVERVIEW_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>
#include <seerep-msgs/query.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "tf.h"

namespace seerep_core
{
class TFOverview
{
public:
  TFOverview();
  TFOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io);
  ~TFOverview();
  std::vector<std::optional<seerep::TransformStamped>> getData(int64_t timesecs, int64_t timenanos,
                                                               std::string parentFrame, std::string childFrame);

  void addDataset(const seerep::TransformStamped& tf);

private:
  void recreateDatasets();
  void addToIndices(std::shared_ptr<seerep_core::TF> tf);

  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;

  std::unordered_map<std::string, std::shared_ptr<seerep_core::TF>> m_datasets;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_TF_OVERVIEW_H_
