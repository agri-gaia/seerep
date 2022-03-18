#ifndef SEEREP_SERVER_SERVER_H_
#define SEEREP_SERVER_SERVER_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// seerep
#include "seerep-server/meta-operations.h"
// #include "seerep-server/receive-sensor-msgs.h"
#include "seerep-server/image-service.h"
#include "seerep-server/point-cloud-service.h"
#include "seerep-server/tf-service.h"

#include <seerep-core/core.h>

namespace seerep_server
{
std::shared_ptr<grpc::Server> createServer(
    const std::string& server_address, seerep_server::MetaOperations* metaOperations,
    seerep_server::TfService* tfService, seerep_server::ImageService* imageService,
    seerep_server::PointCloudService* pointCloudService);  // seerep_server::ReceiveSensorMsgs* receiveSensorMsgs,
                                                           // ,
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_SERVER_H_
