#ifndef SEEREP_CORE_PB_IMAGE_H_
#define SEEREP_CORE_PB_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep_msgs/image.pb.h>
#include <seerep_msgs/query.pb.h>

// seerep_core_msgs
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>

// seerep-hdf5-pb
#include <seerep-hdf5-pb/hdf5-pb-image.h>

// seerep-core
#include <seerep_com/image-service.grpc.pb.h>
#include <seerep_com/image_service.grpc.pb.h>
#include <seerep_core/core.h>

#include "seerep-core-pb/core-pb-conversion.h"

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_core_pb
{
class CorePbImage
{
public:
  CorePbImage(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CorePbImage();

  void getData(const seerep::pb::Query& query, grpc::ServerWriter<seerep::pb::Image>* writer);
  boost::uuids::uuid addData(const seerep::pb::Image& img);

private:
  void getFileAccessorFromCore(boost::uuids::uuid project);
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> getHdf5(boost::uuids::uuid project);
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
  /** the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_IMAGE_H_
