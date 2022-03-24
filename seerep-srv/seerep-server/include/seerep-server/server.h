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
#include "seerep-server/fb-image-service.h"

#include <seerep-core/core.h>

namespace seerep_server
{
std::shared_ptr<grpc::Server> createServerPb(
    const std::string& server_address, seerep_server::PbMetaOperations* metaOperations,
    seerep_server::PbTfService* tfService, seerep_server::PbImageService* imageService,
    seerep_server::PbPointCloudService* pointCloudService);  // seerep_server::PbReceiveSensorMsgs* receiveSensorMsgs,
                                                             // ,
std::shared_ptr<grpc::Server> createServerFb(const std::string& server_address,
                                             seerep_server::FbMetaOperations* metaOperations,
                                             seerep_server::FbImageService* imageService);
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_SERVER_H_
