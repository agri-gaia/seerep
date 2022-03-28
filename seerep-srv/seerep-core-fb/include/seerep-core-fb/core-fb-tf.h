#ifndef SEEREP_CORE_FB_TF_H_
#define SEEREP_CORE_FB_TF_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/transform_stamped_generated.h>
#include <seerep-msgs/transform_stamped_query_generated.h>
// seerep-core-msgs
#include <seerep-msgs/query-tf.h>
// ros
#include <geometry_msgs/TransformStamped.h>
// seerep-hdf5-fb
#include <seerep-hdf5-fb/hdf5-fb-tf.h>

// seerep-conversion
#include <seerep_ros_conversions_fb/conversions.h>

// seerep-core
#include <seerep-core/core.h>

namespace seerep_core_fb
{
class CoreFbTf
{
public:
  CoreFbTf(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbTf();

  void getData(const seerep::fb::TransformStampedQuery& query,
               flatbuffers::grpc::Message<seerep::fb::TransformStamped>* response);
  void addData(const seerep::fb::TransformStamped& tf);
  std::vector<std::string> getFrames(const boost::uuids::uuid& projectuuid);

private:
  void getFileAccessorFromCore(boost::uuids::uuid project);
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbTf> getHdf5(boost::uuids::uuid project);
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_fb::Hdf5FbTf>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_TF_H_
