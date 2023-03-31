#ifndef SEEREP_CORE_PB_POINTCLOUD_H_
#define SEEREP_CORE_PB_POINTCLOUD_H_

#include <functional>
#include <optional>

// seerep_msgs
#include <seerep_msgs/point_cloud_2.pb.h>
#include <seerep_msgs/query.pb.h>

// seerep_core_msgs
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>

// seerep_hdf5_pb
#include <seerep_hdf5_pb/hdf5_pb_pointcloud.h>

// seerep_core
#include <seerep_core/core.h>

#include "core_pb_conversion.h"

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core_pb
{
class CorePbPointCloud
{
public:
  CorePbPointCloud(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CorePbPointCloud();

  std::vector<seerep::pb::PointCloud2> getData(const seerep::pb::Query& query);
  boost::uuids::uuid addData(const seerep::pb::PointCloud2& pc);

private:
  void getFileAccessorFromCore(boost::uuids::uuid project);
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbPointCloud> getHdf5(boost::uuids::uuid project);
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_pb::Hdf5PbPointCloud>,
                     boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_POINTCLOUD_H_
