#ifndef SEEREP_CORE_FB_INSTANE_H_
#define SEEREP_CORE_FB_INSTANE_H_

#include <functional>
#include <optional>

#include "core-fb-conversion.h"

// seerep-msgs
#include <seerep_msgs/query_instance_generated.h>
#include <seerep_msgs/uuids_per_project_generated.h>

// seerep-core-msgs
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>

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
/**
 * @brief This class is the center piece between the gRPC interface, the core and the hdf5-io for instances
 */
class CoreFbInstance
{
public:
  /**
   * @brief Constructs the instance specific object based on the general core
   * @param seerepCore a shared pointer to the general core
   */
  CoreFbInstance(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbInstance();

  /**
   * @brief Function to instances images
   * @param request the flatbuffer query
   * @param response the gRPC flatbuffer message containing the uuids of the instances per project
   */
  void getInstances(const flatbuffers::grpc::Message<seerep::fb::QueryInstance>* request,
                    flatbuffers::grpc::Message<seerep::fb::UuidsPerProject>* response);

private:
  /** @brief a shared pointer to the general core */
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_INSTANE_H_
