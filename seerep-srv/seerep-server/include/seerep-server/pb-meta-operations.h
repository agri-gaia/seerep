#ifndef SEEREP_SERVER_META_OPERATIONS_H_
#define SEEREP_SERVER_META_OPERATIONS_H_

// seerep
#include <seerep-com/meta-operations.grpc.pb.h>
#include <seerep-core/core.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class PbMetaOperations final : public seerep::MetaOperations::Service
{
public:
  PbMetaOperations(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status CreateProject(grpc::ServerContext* context, const seerep::ProjectCreation* request,
                             seerep::ProjectInfo* response);
  grpc::Status GetProjects(grpc::ServerContext* context, const google::protobuf::Empty* request,
                           seerep::ProjectInfos* response);
  grpc::Status GetOverallTimeInterval(grpc::ServerContext* context, const seerep::UuidDatatypePair* request,
                                      seerep::TimeInterval* response);
  grpc::Status GetOverallBoundingBox(grpc::ServerContext* context, const seerep::UuidDatatypePair* request,
                                     seerep::Boundingbox* response);

private:
  std::shared_ptr<seerep_core::Core> seerepCore;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_META_OPERATIONS_H_
