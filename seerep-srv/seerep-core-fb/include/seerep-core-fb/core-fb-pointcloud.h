#ifndef SEEREP_CORE_FB_POINTCLOUD_H_
#define SEEREP_CORE_FB_POINTCLOUD_H_

#include <functional>
#include <optional>

#include "core-fb-conversion.h"
#include "core-fb-general.h"

// seerep-msgs
#include <seerep-msgs/point_cloud_2_generated.h>
#include <seerep-msgs/query_generated.h>
// seerep-core-msgs
#include <seerep-msgs/query-result.h>
#include <seerep-msgs/query.h>
// seerep-hdf5-pb
#include <seerep-hdf5-fb/hdf5-fb-pointcloud.h>
// seerep-core
#include <seerep-core/core.h>

// add as temporary fix
#include <seerep-com/point_cloud_service.grpc.fb.h>
// uuid

#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core_fb
{
class CoreFbPointCloud
{
public:
  CoreFbPointCloud(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbPointCloud();

  void getData(const seerep::fb::Query* query,
               grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* const writer);
  boost::uuids::uuid addData(const seerep::fb::PointCloud2& pc);

private:
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbPointCloud> getHdf5(boost::uuids::uuid project);
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_fb::Hdf5FbPointCloud>,
                     boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_POINTCLOUD_H_
