#include "seerep_server/server.h"

extern const char* GIT_TAG;
extern const char* GIT_REV;
extern const char* GIT_BRANCH;

namespace seerep_server
{
server::server(int argc, char** argv)
{
  signal(SIGINT, signalHandler);
  parseProgramOptions(argc, argv);
  initLogging();
  logTimeZone();
  logServerVersion();
  createGrpcServer();
}

void server::serve()
{
  m_grpcServer->Wait();
}

void server::signalHandler(int signum)
{
  std::cout << "Interrupt signal (" << signum << ") received. Flushing log file." << std::endl;
  boost::log::core::get()->flush();
  exit(EXIT_SUCCESS);
}

void server::parseProgramOptions(int argc, char** argv)
{
  try
  {
    // Declare a group of options that will be allowed only on command line
    boost::program_options::options_description generic("Generic options");
    generic.add_options()("version,v", "Get a version string")("help", "Help message")(
        "config,c", boost::program_options::value<std::string>(), "Path to a configuration file");

    // Declare a group of options that will be allowed both on command line and in config file
    boost::program_options::options_description config("Configuration");
    config.add_options()("data-folder,D",
                         boost::program_options::value<std::string>()->default_value(std::filesystem::current_path()),
                         "Data storage folder")(
        "log-path,L", boost::program_options::value<std::string>()->default_value(std::filesystem::current_path()),
        "Path to store the logs")("log-level", boost::program_options::value<std::string>()->default_value("info"),
                                  "log-level [trace, debug, info, warning, error, fatal]")(
        "port,p", boost::program_options::value<std::string>()->default_value("9090"), "gRPC port to use");

    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic).add(config);

    boost::program_options::options_description config_file_options;
    config_file_options.add(config);

    boost::program_options::options_description visible("Allowed options");
    visible.add(generic).add(config);

    store(boost::program_options::parse_environment(config, environmentVariabeNameMapper), m_programOptionsMap);

    store(boost::program_options::command_line_parser(argc, argv).options(cmdline_options).run(), m_programOptionsMap);
    notify(m_programOptionsMap);

    if (m_programOptionsMap.count("config"))
    {
      std::ifstream ifs(m_programOptionsMap["config"].as<std::string>().c_str());
      if (!ifs)
      {
        std::cout << "can not open config file: " << m_programOptionsMap["config"].as<std::string>() << std::endl;
        exit(EXIT_FAILURE);
      }
      else
      {
        store(parse_config_file(ifs, config_file_options), m_programOptionsMap);
        notify(m_programOptionsMap);
      }
    }

    if (m_programOptionsMap.count("help"))
    {
      std::cout << visible << std::endl;
      exit(EXIT_SUCCESS);
    }

    if (m_programOptionsMap.count("version"))
    {
      std::cout << "SEEREP, version " << GIT_TAG << std::endl;
      exit(EXIT_SUCCESS);
    }
  }
  catch (std::exception& e)
  {
    std::cout << "could not parse programming options" << std::endl;
    std::cout << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }
}

std::string server::environmentVariabeNameMapper(std::string envName)
{
  if (envName == "SEEREP_DATA_FOLDER")
  {
    return "data-folder";
  }
  else if (envName == "SEEREP_LOG_PATH")
  {
    return "log-path";
  }
  else if (envName == "SEEREP_LOG_LEVEL")
  {
    return "log-level";
  }
  else if (envName == "SEEREP_PORT")
  {
    return "port";
  }
  else
  {
    return "";
  }
}

void server::initLogging()
{
  boost::log::add_common_attributes();
  boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");
  setSeverityLevel();

  std::string logPath = initFileLogging();
  initConsoleLogging();
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "Initialized logging";
  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "The used logging folder is: " << logPath;
  }
  catch (std::exception& e)
  {
    // catch exception if there is an error with file logging
    // remove file logging (=remove all sinks add console logging again)
    boost::log::core::get()->remove_all_sinks();
    initConsoleLogging();
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << "File logging exeption: " << e.what();
  }
}

void server::logTimeZone()
{
  time_t time = 0;
  struct tm timeStruct;
  char buf[16];
  localtime_r(&time, &timeStruct);
  strftime(buf, sizeof(buf), "%Z", &timeStruct);
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "Current timezone: " << buf;
}

void server::logServerVersion()
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "SEEREP version: " << GIT_TAG;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "SEEREP git commit hash: " << GIT_REV;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "SEEREP git branch: " << GIT_BRANCH;
}

void server::setSeverityLevel()
{
  std::string logLevel = m_programOptionsMap.at("log-level").as<std::string>();

  if (logLevel == "trace")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::trace);
  }
  else if (logLevel == "debug")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::debug);
  }
  else if (logLevel == "info")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);
  }
  else if (logLevel == "warning")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::warning);
  }
  else if (logLevel == "error")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::error);
  }
  else if (logLevel == "fatal")
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::fatal);
  }
  else
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);
  }
}

std::string server::initFileLogging()
{
  std::string logPath;
  if (m_programOptionsMap.count("log-path"))
  {
    logPath = m_programOptionsMap.at("log-path").as<std::string>();
  }
  if (!logPath.empty())
  {
    boost::log::add_file_log(boost::log::keywords::file_name = logPath + "seerep_%N.log",
                             boost::log::keywords::rotation_size = 10 * 1024 * 1024,
                             boost::log::keywords::time_based_rotation =
                                 boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
                             boost::log::keywords::format = "[%TimeStamp%]<%Severity%>: %Message%");
  }

  return logPath;
}

void server::initConsoleLogging()
{
  boost::log::add_console_log(std::cout, boost::log::keywords::format = "[%TimeStamp%]<%Severity%>: %Message%");
}

void server::createGrpcServer()
{
  m_seerepCore = std::make_shared<seerep_core::Core>(getDataFolder());

  // create server builder and set server address / port
  std::string serverAddress = "[::]:" + m_programOptionsMap.at("port").as<std::string>();
  grpc::ServerBuilder serverBuilder;
  serverBuilder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());
  serverBuilder.SetMaxReceiveMessageSize(messageSize);
  serverBuilder.SetMaxSendMessageSize(messageSize);

  // add protobuf (Pb) services
  addServicesPb(serverBuilder);
  // add flatbuffer (Fb) services
  addServicesFb(serverBuilder);

  // create the server and serve
  m_grpcServer = std::shared_ptr<grpc::Server>(serverBuilder.BuildAndStart());
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "Serving gRPC Server on \"" << serverAddress << "\"";
}

std::string server::getDataFolder()
{
  try
  {
    std::string dataFolder = m_programOptionsMap.at("data-folder").as<std::string>();

    // append '/' if not path does not end with it
    if (!dataFolder.empty() && dataFolder.back() != '/')
    {
      dataFolder += '/';
    }
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "The used data folder is: " << dataFolder;

    std::filesystem::create_directories(dataFolder);
    return dataFolder;
  }
  catch (std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::fatal) << e.what();
    exit(EXIT_FAILURE);
  }
}

void server::addServicesPb(grpc::ServerBuilder& server_builder)
{
  createServicesPb();
  server_builder.RegisterService(&*m_metaOperationsPb);
  server_builder.RegisterService(&*m_tfServicePb);
  server_builder.RegisterService(&*m_imageServicePb);
  server_builder.RegisterService(&*m_pointCloudServicePb);
  server_builder.RegisterService(&*m_cameraIntrinsicsServicePb);
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "Addded Protocol Buffers gRPC services";
}

void server::addServicesFb(grpc::ServerBuilder& server_builder)
{
  createServicesFb();
  server_builder.RegisterService(&*m_metaOperationsFb);
  server_builder.RegisterService(&*m_tfServiceFb);
  server_builder.RegisterService(&*m_instanceServiceFb);
  server_builder.RegisterService(&*m_imageServiceFb);
  server_builder.RegisterService(&*m_pointServiceFb);
  server_builder.RegisterService(&*m_pointCloudServiceFb);
  server_builder.RegisterService(&*m_cameraIntrinsicsServiceFb);
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "Added Flatbuffers gRPC services";
}

void server::createServicesPb()
{
  m_metaOperationsPb = std::make_shared<seerep_server::PbMetaOperations>(m_seerepCore);
  m_tfServicePb = std::make_shared<seerep_server::PbTfService>(m_seerepCore);
  m_imageServicePb = std::make_shared<seerep_server::PbImageService>(m_seerepCore);
  m_pointCloudServicePb = std::make_shared<seerep_server::PbPointCloudService>(m_seerepCore);
  m_cameraIntrinsicsServicePb = std::make_shared<seerep_server::PbCameraIntrinsicsService>(m_seerepCore);
}

void server::createServicesFb()
{
  m_metaOperationsFb = std::make_shared<seerep_server::FbMetaOperations>(m_seerepCore);
  m_tfServiceFb = std::make_shared<seerep_server::FbTfService>(m_seerepCore);
  m_instanceServiceFb = std::make_shared<seerep_server::FbInstanceService>(m_seerepCore);
  m_imageServiceFb = std::make_shared<seerep_server::FbImageService>(m_seerepCore);
  m_pointServiceFb = std::make_shared<seerep_server::FbPointService>(m_seerepCore);
  m_pointCloudServiceFb = std::make_shared<seerep_server::FbPointCloudService>(m_seerepCore);
  m_cameraIntrinsicsServiceFb = std::make_shared<seerep_server::FbCameraIntrinsicsService>(m_seerepCore);
}

} /* namespace seerep_server */

int main(int argc, char** argv)
{
  seerep_server::server server(argc, argv);
  server.serve();

  return EXIT_SUCCESS;
}
