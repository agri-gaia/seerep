#ifndef SEEREP_CORE_PB_TF_H_
#define SEEREP_CORE_PB_TF_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep_msgs/transform_stamped.pb.h>
#include <seerep_msgs/transform_stamped_query.pb.h>

// seerep_core_msgs
#include <seerep_msgs/query_tf.h>

// ros
#include <geometry_msgs/TransformStamped.h>

// seerep_hdf5_pb
#include <seerep_hdf5_pb/hdf5_pb_tf.h>

// seerep_conversion
#include <seerep_ros_conversions_pb/conversions.h>

// seerep_core
#include <seerep_core/core.h>

#include "seerep-core-pb/core-pb-conversion.h"

namespace seerep_core_pb
{
class CorePbTf
{
public:
  CorePbTf(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CorePbTf();

  std::optional<seerep::pb::TransformStamped> getData(const seerep::pb::TransformStampedQuery& query);
  void addData(const seerep::pb::TransformStamped& tf);
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
