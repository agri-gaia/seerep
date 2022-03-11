#ifndef SEEREP_CORE_PB_IMAGE_H_
#define SEEREP_CORE_PB_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/image.pb.h>
// seerep-core-msgs
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-result.h>
// seerep-pb-io
#include <seerep-io-pb/io-pb-image.h>
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
class CorePbImage
{
public:
  CorePbImage(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CorePbImage();

  std::vector<seerep::Image> getData(const seerep::Query& query);
  boost::uuids::uuid addData(const seerep::Image& img);

private:
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_io_pb::IoPbImage>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_IMAGE_H_
