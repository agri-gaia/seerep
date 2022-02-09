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
#include "seerep-server/receive-sensor-msgs.h"
#include "seerep-server/imageService.h"
#include "seerep-server/pointCloudService.h"
#include "seerep-server/tfService.h"

#include <seerep-core/project-overview.h>

namespace seerep_server
{
std::shared_ptr<grpc::Server>
createServer(const std::string& server_address, seerep_server::MetaOperations* metaOperations,
             seerep_server::ReceiveSensorMsgs* receiveSensorMsgs, seerep_server::ImageService* imageService,
             seerep_server::PointCloudService* pointCloudService, seerep_server::TfService* tfService);
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_SERVER_H_
