#ifndef SEEREP_SERVER_TF_SERVICE_H_
#define SEEREP_SERVER_TF_SERVICE_H_

// seerep
#include <seerep-com/tf-service.grpc.pb.h>
#include <seerep-core-pb/core-pb-tf.h>
#include <seerep-core/core.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class PbTfService final : public seerep::pb::TfService::Service
{
public:
  PbTfService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status TransferTransformStamped(grpc::ServerContext* context, const seerep::pb::TransformStamped* transform,
                                        seerep::pb::ServerResponse* response);
  grpc::Status GetFrames(grpc::ServerContext* context, const seerep::pb::FrameQuery* frameQuery,
                         seerep::pb::FrameInfos* response);
  grpc::Status GetTransformStamped(grpc::ServerContext* context,
                                   const seerep::pb::TransformStampedQuery* transformQuery,
                                   seerep::pb::TransformStamped* response);

private:
  std::shared_ptr<seerep_core_pb::CorePbTf> tfPb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_TF_SERVICE_H_
