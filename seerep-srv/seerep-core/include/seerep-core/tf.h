#ifndef SEEREP_CORE_TF_H_
#define SEEREP_CORE_TF_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>
// seerep-hdf5
#include <seerep-hdf5/tf-io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

namespace seerep_core
{
class TF
{
public:
  TF(std::shared_ptr<seerep_hdf5::TfIO> hdf5_io, const seerep::TransformStamped& tf);
  TF(std::shared_ptr<seerep_hdf5::TfIO> hdf5_io, const std::string& id);
  ~TF();

  void addData(const seerep::TransformStamped& tf);
  std::optional<std::vector<seerep::TransformStamped>> getData();
  std::string getParentFrame();
  std::string getChildFrame();
  std::string getID();

  static std::string idFromFrameNames(const std::string& parentframe, const std::string& childframe);

private:
  std::shared_ptr<seerep_hdf5::TfIO> m_hdf5_io;
  const std::string m_id;
  std::string m_parentframe;
  std::string m_childframe;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_TF_H_
