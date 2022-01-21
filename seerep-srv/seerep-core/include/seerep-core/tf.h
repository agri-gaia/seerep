#ifndef SEEREP_CORE_TF_H_
#define SEEREP_CORE_TF_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

namespace seerep_core
{
class TF
{
public:
  TF(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const seerep::TransformStamped& tf);
  TF(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const std::string& id);
  ~TF();

  std::optional<std::vector<seerep::TransformStamped>> getData();
  std::string getID();

private:
  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;
  const std::string m_id;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_TF_H_
