#ifndef SEEREP_SERVER_FB_TF_SERVICE_H_
#define SEEREP_SERVER_FB_TF_SERVICE_H_

// seerep
#include <seerep-core-fb/core-fb-tf.h>
#include <seerep_com/tf_service.grpc.fb.h>
#include <seerep_core/core.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class FbTfService final : public seerep::fb::TfService::Service
{
public:
  FbTfService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status
  TransferTransformStamped(grpc::ServerContext* context,
                           grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::TransformStamped>>* reader,
                           flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response);
  grpc::Status GetFrames(grpc::ServerContext* context,
                         const flatbuffers::grpc::Message<seerep::fb::FrameQuery>* request,
                         flatbuffers::grpc::Message<seerep::fb::FrameInfos>* response);
  grpc::Status GetTransformStamped(grpc::ServerContext* context,
                                   const flatbuffers::grpc::Message<seerep::fb::TransformStampedQuery>* request,
                                   flatbuffers::grpc::Message<seerep::fb::TransformStamped>* response);

private:
  std::shared_ptr<seerep_core_fb::CoreFbTf> tfFb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_TF_SERVICE_H_
