#ifndef SEEREP_CORE_PB_IMAGE_H_
#define SEEREP_CORE_PB_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/image.pb.h>
#include <seerep-msgs/query.pb.h>
// seerep-core-msgs
#include <seerep-msgs/query-result.h>
#include <seerep-msgs/query.h>
// seerep-hdf5-pb
#include <seerep-hdf5-pb/hdf5-pb-image.h>
// seerep-core
#include <seerep-core/core.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core_pb
{
class CorePbImage
{
public:
  CorePbImage(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CorePbImage();

  std::vector<seerep::Image> getData(const seerep::Query& query);
  boost::uuids::uuid addData(const seerep::Image& img);

private:
  void getFileAccessorFromCore(boost::uuids::uuid project);
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> getHdf5(boost::uuids::uuid project);
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_IMAGE_H_
