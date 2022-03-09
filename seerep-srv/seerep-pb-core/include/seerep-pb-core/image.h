#ifndef SEEREP_PB_CORE_IMAGE_H_
#define SEEREP_PB_CORE_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/image.pb.h>
#include <seerep-msgs/server_response.pb.h>
// seerep-core-msgs
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-result.h>
// seerep-pb-io
#include <seerep-pb-io/image-io.h>

// seerep-core
#include <seerep-core/seerep-core.h>

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_core_pb
{
class ImagePb
{
public:
  ImagePb(std::shared_ptr<seerep_core::SeerepCore> seerepCore);
  ~ImagePb();

  std::vector<seerep::Image> getData(const seerep::Query& query);
  boost::uuids::uuid addData(const seerep::Image& img);

private:
  std::shared_ptr<seerep_core::SeerepCore> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_pb_io::ImageIO>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_PB_CORE_IMAGE_H_
