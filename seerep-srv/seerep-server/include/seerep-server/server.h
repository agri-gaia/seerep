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
#include "seerep-server/query-data.h"
#include "seerep-server/receive-sensor-msgs.h"

#include <seerep-core/project-overview.h>

namespace seerep_server
{
std::shared_ptr<grpc::Server> createServer(const std::string& server_address,
                                           seerep_server::ReceiveSensorMsgs* receive_sensor_msgs);
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_SERVER_H_
