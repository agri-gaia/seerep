#ifndef SEEREP_SERVER_UTIL_H_
#define SEEREP_SERVER_UTIL_H_

#include <seerep-com/image_service.grpc.fb.h>
#include <seerep-com/point_cloud_service.grpc.pb.h>

namespace seerep_server_util
{
inline void createResponseFb(std::string msg, seerep::fb::TRANSMISSION_STATE state,
                             flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  flatbuffers::grpc::MessageBuilder builder;
  auto msgFb = builder.CreateString(msg);
  seerep::fb::ServerResponseBuilder responseBuilder(builder);
  responseBuilder.add_message(msgFb);
  responseBuilder.add_transmission_state(state);
  auto responseOffset = responseBuilder.Finish();
  builder.Finish(responseOffset);
  *response = builder.ReleaseMessage<seerep::fb::ServerResponse>();
  assert(response->Verify());
}

inline void createResponsePb(std::string msg, seerep::ServerResponse::TransmissionState state,
                             seerep::ServerResponse* response)
{
  response->set_message(msg);
  response->set_transmission_state(state);
}

}  // namespace seerep_server_util
#endif  // SEEREP_SERVER_UTIL_H_
