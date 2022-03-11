#ifndef SEEREP_CORE_PB_POINTCLOUD_H_
#define SEEREP_CORE_PB_POINTCLOUD_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
// seerep-core-msgs
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-result.h>
// seerep-pb-io
#include <seerep-io-pb/io-pb-pointcloud.h>
// seerep-core
#include <seerep-core/core.h>

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_core_pb
{
class CorePbPointCloud
{
public:
  CorePbPointCloud(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CorePbPointCloud();

  std::vector<seerep::PointCloud2> getData(const seerep::Query& query);
  boost::uuids::uuid addData(const seerep::PointCloud2& pc);

private:
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_io_pb::IoPbPointCloud>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_POINTCLOUD_H_
