#ifndef SEEREP_CORE_FB_IMAGE_H_
#define SEEREP_CORE_FB_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query_generated.h>
#include <seerep-msgs/image_generated.h>
// seerep-core-msgs
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-result.h>
// seerep-hdf5-fb
#include <seerep-hdf5-fb/hdf5-fb-image.h>
// seerep-core
#include <seerep-core/core.h>

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_core_fb
{
class CoreFbImage
{
public:
  CoreFbImage(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbImage();

  // std::vector<flatbuffers::Offset<seerep::fb::Image>>
  void getData(const seerep::fb::Query& query,
               grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* const writer);
  boost::uuids::uuid addData(const seerep::fb::Image& img);

private:
  void getFileAccessorFromCore(boost::uuids::uuid project);
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> getHdf5(boost::uuids::uuid project);
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_IMAGE_H_
