#include "seerep-server/server.h"

namespace seerep_server
{
server::server(int argc, char** argv)
{
  std::cout << "Starting seerep server" << std::endl;
  signal(SIGINT, signalHandler);
  std::cout << "trigger signal" << std::endl;
  parseProgramOptions(argc, argv);
  initLogging();

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
    std::cout << "parsing programming options" << argv << std::endl;

    for (i = 0; i < argc - 1; i++)
    {
      std::cout << "options" << argv[i] << std::endl;
    }

    // Declare a group of options that will be
    // allowed only on command line
    boost::program_options::options_description generic("Generic options");
    generic.add_options()("version,v", "print version string")("help", "produce help message")(
        "config,c", boost::program_options::value<std::string>(), "name of a file of a configuration.");

    // Declare a group of options that will be
    // allowed both on command line and in
    // config file
    boost::program_options::options_description config("Configuration");
    config.add_options()("data-folder,D",
                         boost::program_options::value<std::string>()->default_value(std::filesystem::current_path()),
                         "data folder")("log-path,L", boost::program_options::value<std::string>(), "log path")(
        "port,p", boost::program_options::value<std::string>()->default_value("9090"), "gRPC port");

    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic).add(config);

    boost::program_options::options_description config_file_options;
    config_file_options.add(config);

    boost::program_options::options_description visible("Allowed options");
    visible.add(generic).add(config);

    store(boost::program_options::command_line_parser(argc, argv).options(cmdline_options).run(), m_vm);
    notify(m_vm);

    for (const auto& it : m_vm)
    {
      std::cout << it.first.c_str() << " ";
      auto& value = it.second.value();
      if (auto v = boost::any_cast<uint32_t>(&value))
      {
        std::cout << *v;
      }
      else if (auto v = boost::any_cast<std::string>(&value))
      {
        std::cout << *v;
      }
      else
      {
        std::cout << "error";
      }
    }

    if (m_vm.count("config"))
    {
      std::ifstream ifs(m_vm["config"].as<std::string>().c_str());
      if (!ifs)
      {
        std::cout << "can not open config file: " << m_vm["config"].as<std::string>() << std::endl;
        exit(EXIT_FAILURE);
      }
      else
      {
        store(parse_config_file(ifs, config_file_options), m_vm);
        notify(m_vm);
      }
    }

    if (m_vm.count("help"))
    {
      std::cout << visible << std::endl;
      exit(EXIT_SUCCESS);
    }

    if (m_vm.count("version"))
    {
      std::cout << "SEEREP, version 0.0\n";
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

void server::initLogging()
{
  std::string logPath;
  if (m_vm.count("log-path"))
  {
    logPath = m_vm.at("log-path").as<std::string>();
  }

  boost::log::add_common_attributes();
  boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");

  if (!logPath.empty())
  {
    boost::log::add_file_log(boost::log::keywords::file_name = logPath + "seerep_%N.log",
                             boost::log::keywords::rotation_size = 10 * 1024 * 1024,
                             boost::log::keywords::time_based_rotation =
                                 boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
                             boost::log::keywords::format = "[%TimeStamp%]<%Severity%>: %Message%");
  }
  boost::log::add_console_log(std::cout, boost::log::keywords::format = "[%TimeStamp%]<%Severity%>: %Message%");

  boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);

  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "The used logging folder is: " << (logPath.empty() ? "the data folder" : logPath);
  }
  catch (std::exception& e)
  {
    // catch exeption if there is an error with file logging and remove file logging
    boost::log::core::get()->remove_all_sinks();
    boost::log::add_console_log(std::cout, boost::log::keywords::format = "[%TimeStamp%]<%Severity%>: %Message%");
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << "file logging exeption: " << e.what();
  }
}

void server::createGrpcServer()
{
  m_seerepCore = std::make_shared<seerep_core::Core>(getDataFolder());

  // create server builder and set server address / port
  std::string serverAddress = "[::]:" + m_vm.at("port").as<std::string>();
  grpc::ServerBuilder serverBuilder;
  serverBuilder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());

  // add flatbuffer and protobuf services
  addServicesPb(serverBuilder);
  addServicesFb(serverBuilder);

  // create the server and serve
  m_grpcServer = std::shared_ptr<grpc::Server>(serverBuilder.BuildAndStart());
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "serving gRPC Server on \"" << serverAddress << "\"...";
}

std::string server::getDataFolder()
{
  std::string dataFolder = m_vm.at("data-folder").as<std::string>();

  // append '/' if not path does not end with it
  if (!dataFolder.empty() && dataFolder.back() != '/')
  {
    dataFolder += '/';
  }
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "The used data folder is: " << dataFolder;

  try
  {
    std::filesystem::create_directories(dataFolder);
  }
  catch (std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::fatal) << e.what();
    exit(EXIT_FAILURE);
  }

  return dataFolder;
}

void server::addServicesPb(grpc::ServerBuilder& server_builder)
{
  // create services
  createServicesPb();
  // add services
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "add the protobuf gRPC services...";
  server_builder.RegisterService(&*m_metaOperationsPb);
  server_builder.RegisterService(&*m_tfServicePb);
  server_builder.RegisterService(&*m_imageServicePb);
  server_builder.RegisterService(&*m_pointCloudServicePb);
  // server_builder.RegisterService(receiveSensorMsgs);
}

void server::addServicesFb(grpc::ServerBuilder& server_builder)
{
  // create services
  createServicesFb();
  // add services
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "add the flatbuffer gRPC services...";
  server_builder.RegisterService(&*m_metaOperationsFb);
  server_builder.RegisterService(&*m_tfServiceFb);
  server_builder.RegisterService(&*m_instanceServiceFb);
  server_builder.RegisterService(&*m_imageServiceFb);
}

void server::createServicesPb()
{
  m_metaOperationsPb = std::make_shared<seerep_server::PbMetaOperations>(m_seerepCore);
  m_tfServicePb = std::make_shared<seerep_server::PbTfService>(m_seerepCore);
  m_imageServicePb = std::make_shared<seerep_server::PbImageService>(m_seerepCore);
  m_pointCloudServicePb = std::make_shared<seerep_server::PbPointCloudService>(m_seerepCore);
}

void server::createServicesFb()
{
  m_metaOperationsFb = std::make_shared<seerep_server::FbMetaOperations>(m_seerepCore);
  m_tfServiceFb = std::make_shared<seerep_server::FbTfService>(m_seerepCore);
  m_instanceServiceFb = std::make_shared<seerep_server::FbInstanceService>(m_seerepCore);
  m_imageServiceFb = std::make_shared<seerep_server::FbImageService>(m_seerepCore);
}

} /* namespace seerep_server */

int main(int argc, char** argv)
{
  seerep_server::server server(argc, argv);
  server.serve();

  return EXIT_SUCCESS;
}
