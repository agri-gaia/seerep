#ifndef SEEREP_SERVER_SERVER_H_
#define SEEREP_SERVER_SERVER_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// seerep
#include "seerep-server/pb-meta-operations.h"
// #include "seerep-server/pb-receive-sensor-msgs.h"
#include "seerep-server/pb-image-service.h"
#include "seerep-server/pb-point-cloud-service.h"
#include "seerep-server/pb-tf-service.h"

#include "seerep-server/fb-meta-operations.h"
#include "seerep-server/fb-tf-service.h"
#include "seerep-server/fb-image-service.h"

#include <seerep-core/core.h>

namespace seerep_server
{
class server
{
public:
  server(std::shared_ptr<seerep_core::Core> seerepCore);

  void addServicesPb(grpc::ServerBuilder& server_builder);

  void addServicesFb(grpc::ServerBuilder& server_builder);

private:
  void createServicesPb();
  void createServicesFb();

  std::shared_ptr<seerep_core::Core> m_seerepCore;

  // protobuf services
  std::shared_ptr<seerep_server::PbMetaOperations> m_metaOperationsPb;
  std::shared_ptr<seerep_server::PbTfService> m_tfServicePb;
  std::shared_ptr<seerep_server::PbImageService> m_imageServicePb;
  std::shared_ptr<seerep_server::PbPointCloudService> m_pointCloudServicePb;

  // flatbuffer services
  std::shared_ptr<seerep_server::FbMetaOperations> m_metaOperationsFb;
  std::shared_ptr<seerep_server::FbTfService> m_tfServiceFb;
  std::shared_ptr<seerep_server::FbImageService> m_imageServiceFb;

  // flatbuffer services
};
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_SERVER_H_
