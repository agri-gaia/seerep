#ifndef SEEREP_SERVER_H_
#define SEEREP_SERVER_H_

#include <signal.h>

#include <fstream>

// grpc
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

// seerep
#include <seerep_core/core.h>

// seerep_server
#include "seerep_server/fb_cameraintrinsics_service.h"
#include "seerep_server/fb_image_service.h"
#include "seerep_server/fb_instance_service.h"
#include "seerep_server/fb_meta_operations.h"
#include "seerep_server/fb_point_cloud_service.h"
#include "seerep_server/fb_point_service.h"
#include "seerep_server/fb_tf_service.h"
#include "seerep_server/pb_camera_intrinsics_service.h"
#include "seerep_server/pb_image_service.h"
#include "seerep_server/pb_meta_operations.h"
#include "seerep_server/pb_point_cloud_service.h"
#include "seerep_server/pb_tf_service.h"

// logging
#include <boost/log/core.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>

// program options / cmd line args
#include <boost/program_options.hpp>

namespace seerep_server
{
/**
 * @brief This class is the main class of SEEREP. It creates everything else and
 * serves the gRPC interface.
 *
 * It initialises the logging and parses the configuration from command line or
 * cfg file.
 *
 * The various flatbuffer and protobuf services are created here and served via
 * the server.
 */
class server
{
public:
  /**
   * @brief Constructs the services and the server
   * @param argc argument count of the command line arguments
   * @param argv argument vector of the command line arguments
   */
  server(int argc, char** argv);
  /**
   * @brief serves the services and waits for queries
   */
  void serve();

private:
  /**
   * @brief signal handler to flush the logging to file on SIGTERM
   */
  static void signalHandler(int signum);
  /**
   * @brief parses the command line arguments
   * @param argc argument count of the command line arguments
   * @param argv argument vector of the command line arguments
   */
  void parseProgramOptions(int argc, char** argv);
  /**
   * @brief maps the environment variable names to the command line names of the options
   *
   * @return std::string the mapped name matching the command line options name
   */
  static std::string environmentVariabeNameMapper(std::string envName);
  /**
   * @brief initializes the logging
   *
   * Logs are stored in a log file and streamed to stdout
   */
  void initLogging();

  /**
   * @brief Logs the used time zone at server startup
   */
  void logTimeZone();

  /**
   * @brief Logs the version of the server
   */
  void logServerVersion();

  /**
   * @brief sets the severity level of the logging
   */
  void setSeverityLevel();

  /**
   * @brief initializes the file logging
   * @return the folder in which the logs are stored
   */
  std::string initFileLogging();

  /**
   * @brief initializes the console logging
   */
  void initConsoleLogging();

  /**
   * @brief creates the gRPC server
   *
   * The SEEREP core, the flatbuffer services and the protobuf services are
   * created and used to create the server
   */
  void createGrpcServer();
  /**
   * @brief Returns the path to the data folder. The folder is created if not existing
   * @return path to the data folder
   */
  std::string getDataFolder();

  /**
   * @brief calls the create function and registers the protobuf services at the server
   * @param server_builder the ServerBuilder which is used to build the server
   * and needed to add the services
   */
  void addServicesPb(grpc::ServerBuilder& server_builder);
  /**
   * @brief calls the create function and registers the flatbuffer services at the server
   * @param server_builder the ServerBuilder which is used to build the server
   * and needed to add the services
   */
  void addServicesFb(grpc::ServerBuilder& server_builder);

  /**
   * @brief creates the protobuf services
   */
  void createServicesPb();
  /**
   * @brief creates the flatbuffer services
   */
  void createServicesFb();

  /** @brief the value map of the program options from commandline or cfg file*/
  boost::program_options::variables_map m_programOptionsMap;

  /** @brief the seerep core which deals with the data indexing*/
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  /** @brief the gRPC server which serves the gRPC services*/
  std::shared_ptr<grpc::Server> m_grpcServer;

  //** @brief the maximum size of grpc messages in bytes*/
  inline static const int messageSize = 1 * 1024 * 1024 * 1024;

  /** @brief the protobuf service for meta operations*/
  std::shared_ptr<seerep_server::PbMetaOperations> m_metaOperationsPb;
  /** @brief the protobuf service for transformation related queries*/
  std::shared_ptr<seerep_server::PbTfService> m_tfServicePb;
  /** @brief the protobuf service for image related queries*/
  std::shared_ptr<seerep_server::PbImageService> m_imageServicePb;
  /** @brief the protobuf service for point cloud related queries*/
  std::shared_ptr<seerep_server::PbPointCloudService> m_pointCloudServicePb;
  /** @brief the protobuf service for camera intrinsics related queries*/
  std::shared_ptr<seerep_server::PbCameraIntrinsicsService>
      m_cameraIntrinsicsServicePb;

  /** @brief the flatbuffer service for meta operations*/
  std::shared_ptr<seerep_server::FbMetaOperations> m_metaOperationsFb;
  /** @brief the flatbuffer service for transformation related queries*/
  std::shared_ptr<seerep_server::FbTfService> m_tfServiceFb;
  /** @brief the flatbuffer service for image related queries*/
  std::shared_ptr<seerep_server::FbImageService> m_imageServiceFb;
  /** @brief the flatbuffer service for point related queries*/
  std::shared_ptr<seerep_server::FbPointService> m_pointServiceFb;
  /** @brief the flatbuffer service for instances related queries*/
  std::shared_ptr<seerep_server::FbInstanceService> m_instanceServiceFb;
  /** @brief the flatbuffer service for point clouds */
  std::shared_ptr<seerep_server::FbPointCloudService> m_pointCloudServiceFb;
  /** @brief the flatbuffer service for camera intrinsics */
  std::shared_ptr<seerep_server::FbCameraIntrinsicsService>
      m_cameraIntrinsicsServiceFb;

  /** @brief the logger object for logging to file and stdout*/
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_H_
