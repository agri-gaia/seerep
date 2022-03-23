#ifndef SEEREP_SERVER_FB_META_OPERATIONS_H_
#define SEEREP_SERVER_FB_META_OPERATIONS_H_

// seerep
#include <seerep-com/meta-operations.grpc.fb.h>
#include <seerep-core/core.h>

namespace seerep_server
{
class FbMetaOperations final : public seerep::fb::MetaOperations::Service
{
public:
  FbMetaOperations(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status CreateProject(grpc::ServerContext* context,
                             const flatbuffers::grpc::Message<seerep::fb::ProjectCreation>* request,
                             flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* response);
  grpc::Status GetProjects(grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                           flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response);

private:
  std::shared_ptr<seerep_core::Core> seerepCore;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_META_OPERATIONS_H_
