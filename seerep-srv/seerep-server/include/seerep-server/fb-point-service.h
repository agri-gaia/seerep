#ifndef SEEREP_SERVER_FB_POINT_SERVICE_H_
#define SEEREP_SERVER_FB_POINT_SERVICE_H_

// seerep
#include <seerep-com/point_service.grpc.fb.h>
#include <seerep-core-fb/core-fb-point.h>
#include <seerep-core/core.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class FbPointService final : public seerep::fb::PointService::Service
{
public:
  FbPointService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status GetPoint(grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                        grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointStamped>>* writer) override;
  grpc::Status TransferPoint(grpc::ServerContext* context,
                             grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::PointStamped>>* reader,
                             flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;
  grpc::Status AddAttribute(grpc::ServerContext* context,
                            grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::AttributesStamped>>* reader,
                            flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;

private:
  void createResponse(std::string msg, seerep::fb::TRANSMISSION_STATE state,
                      flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response);

  std::shared_ptr<seerep_core_fb::CoreFbPoint> pointFb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_POINT_SERVICE_H_
