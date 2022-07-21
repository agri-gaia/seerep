#ifndef SEEREP_CORE_PB_TF_H_
#define SEEREP_CORE_PB_TF_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>
#include <seerep-msgs/transform_stamped_query.pb.h>
// seerep-core-msgs
#include <seerep-msgs/query-tf.h>
// ros
#include <geometry_msgs/TransformStamped.h>
// seerep-hdf5-pb
#include <seerep-hdf5-pb/hdf5-pb-tf.h>

// seerep-conversion
#include <seerep_ros_conversions_pb/conversions.h>

// seerep-core
#include <seerep-core/core.h>

#include "seerep-core-pb/core-pb-conversion.h"

namespace seerep_core_pb
{
class CorePbTf
{
public:
  CorePbTf(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CorePbTf();

  std::optional<seerep::TransformStamped> getData(const seerep::TransformStampedQuery& query);
  void addData(const seerep::TransformStamped& tf);
  std::vector<std::string> getFrames(const boost::uuids::uuid& projectuuid);

private:
  void getFileAccessorFromCore(boost::uuids::uuid project);
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbTf> getHdf5(boost::uuids::uuid project);
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_pb::Hdf5PbTf>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_TF_H_
