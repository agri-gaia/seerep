#ifndef SEEREP_CORE_FB_INSTANE_H_
#define SEEREP_CORE_FB_INSTANE_H_

#include <functional>
#include <optional>

#include "core-fb-conversion.h"

// seerep-msgs
#include <seerep-msgs/query_generated.h>
#include <seerep-msgs/uuids_per_project_generated.h>
// seerep-core-msgs
#include <seerep-msgs/query-result.h>
#include <seerep-msgs/query.h>
// seerep-hdf5-fb
//

// seerep-core
#include <seerep-core/core.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core_fb
{
class CoreFbInstance
{
public:
  CoreFbInstance(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbInstance();

  void getInstances(const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                    flatbuffers::grpc::Message<seerep::fb::UuidsPerProject>* response);

private:
  std::shared_ptr<seerep_core::Core> m_seerepCore;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_INSTANE_H_
