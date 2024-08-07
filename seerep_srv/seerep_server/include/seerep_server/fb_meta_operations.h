#ifndef SEEREP_SERVER_FB_META_OPERATIONS_H_
#define SEEREP_SERVER_FB_META_OPERATIONS_H_

// seerep
#include <seerep_com/meta_operations.grpc.fb.h>
#include <seerep_core/core.h>
#include <seerep_core_fb/core_fb_conversion.h>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
namespace seerep_server
{
class FbMetaOperations final : public seerep::fb::MetaOperations::Service
{
public:
  FbMetaOperations(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status CreateProject(
      grpc::ServerContext* context,
      const flatbuffers::grpc::Message<seerep::fb::ProjectCreation>* request,
      flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* response) override;
  grpc::Status GetProjects(
      grpc::ServerContext* context,
      const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
      flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response) override;
  grpc::Status LoadProjects(
      grpc::ServerContext* context,
      const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
      flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response) override;
  grpc::Status DeleteProject(
      grpc::ServerContext* context,
      const flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* request,
      flatbuffers::grpc::Message<seerep::fb::Empty>* response) override;
  grpc::Status GetOverallTimeInterval(
      grpc::ServerContext* context,
      const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
      flatbuffers::grpc::Message<seerep::fb::TimeInterval>* response) override;
  grpc::Status GetOverallBoundingBox(
      grpc::ServerContext* context,
      const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
      flatbuffers::grpc::Message<seerep::fb::Boundingbox>* response) override;
  grpc::Status GetAllCategories(
      grpc::ServerContext* context,
      const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
      flatbuffers::grpc::Message<seerep::fb::StringVector>* response) override;
  grpc::Status GetAllLabels(
      grpc::ServerContext* context,
      const flatbuffers::grpc::Message<seerep::fb::UuidDatatypeWithCategory>*
          request,
      flatbuffers::grpc::Message<seerep::fb::StringVector>* response) override;

private:
  std::shared_ptr<seerep_core::Core> seerepCore;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_META_OPERATIONS_H_
