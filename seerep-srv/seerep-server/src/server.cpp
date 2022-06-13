#include "seerep-server/server.h"

namespace seerep_server
{
server::server(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
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

void initLogging(std::string logPath)
{
  boost::log::add_common_attributes();
  if (!logPath.empty())
  {
    boost::log::add_file_log(boost::log::keywords::file_name = logPath + "seerep_%N.log",
                             boost::log::keywords::rotation_size = 10 * 1024 * 1024,
                             boost::log::keywords::time_based_rotation =
                                 boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
                             boost::log::keywords::format = "[%TimeStamp%]: %Message%");
  }
  boost::log::add_console_log(std::cout, boost::log::keywords::format = "[%TimeStamp%]: %Message%");

  boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);
}

std::optional<boost::program_options::variables_map> parseProgramOptions(int argc, char** argv)
{
  boost::program_options::variables_map vm;
  try
  {
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

    store(boost::program_options::command_line_parser(argc, argv).options(cmdline_options).run(), vm);
    notify(vm);

    if (vm.count("config"))
    {
      std::ifstream ifs(vm["config"].as<std::string>().c_str());
      if (!ifs)
      {
        std::cout << "can not open config file: " << vm["config"].as<std::string>() << std::endl;
        return std::nullopt;
      }
      else
      {
        store(parse_config_file(ifs, config_file_options), vm);
        notify(vm);
      }
    }

    if (vm.count("help"))
    {
      std::cout << visible << std::endl;
      return std::nullopt;
    }

    if (vm.count("version"))
    {
      std::cout << "SEEREP, version 0.0\n";
      return std::nullopt;
    }
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return std::nullopt;
  }

  return vm;
}

int main(int argc, char** argv)
{
  auto vm = parseProgramOptions(argc, argv);

  if (!vm)
  {
    return EXIT_SUCCESS;
  }

  std::string logPath;

  if (vm.value().count("log-path"))
  {
    logPath = vm.value().at("log-path").as<std::string>();
  }

  boost::log::sources::severity_logger<boost::log::trivial::severity_level> logger;
  initLogging(logPath);

  // use the path from argument
  std::string datafolder = vm.value().at("data-folder").as<std::string>();
  // append '/' if not path does not end with it
  if (!datafolder.empty() && datafolder.back() != '/')
  {
    datafolder += '/';
  }

  BOOST_LOG_SEV(logger, boost::log::trivial::severity_level::info) << "The used data folder is: ";
  auto seerepCore = std::make_shared<seerep_core::Core>(datafolder);

  // create server builder and set server address / port
  std::string serverAddress = "[::]:" + vm.value().at("port").as<std::string>();
  grpc::ServerBuilder serverBuilder;
  serverBuilder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());

  // add flatbuffer and protobuf services
  seerep_server::server seerepServer(seerepCore);
  seerepServer.addServicesPb(serverBuilder);
  seerepServer.addServicesFb(serverBuilder);

  // create the server and serve
  std::shared_ptr<grpc::Server> grpcServer = std::shared_ptr<grpc::Server>(serverBuilder.BuildAndStart());
  BOOST_LOG_SEV(logger, boost::log::trivial::severity_level::info)
      << "serving gRPC Server on \"" << serverAddress << "\"...";
  grpcServer->Wait();

  return EXIT_SUCCESS;
}
