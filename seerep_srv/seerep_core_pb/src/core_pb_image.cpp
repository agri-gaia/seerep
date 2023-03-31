#include "seerep_core_pb/core_pb_image.h"

namespace seerep_core_pb
{
CorePbImage::CorePbImage(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : m_seerepCore->getProjects())
  {
    getFileAccessorFromCore(projectInfo.uuid);
  }
}

CorePbImage::~CorePbImage()
{
}

void CorePbImage::getData(const seerep::pb::Query& query, grpc::ServerWriter<seerep::pb::Image>* const writer)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore = CorePbConversion::fromPb(query, seerep_core_msgs::Datatype::Image);

  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getDataset(queryCore);

  for (auto project : resultCore.queryResultProjects)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "sending images from project" << boost::lexical_cast<std::string>(project.projectUuid);
    for (auto uuidImg : project.dataOrInstanceUuids)
    {
      auto hdf5io = getHdf5(project.projectUuid);
      std::optional<seerep::pb::Image> image = hdf5io->readImage(boost::lexical_cast<std::string>(uuidImg));
      if (image)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
            << "sending img " << boost::lexical_cast<std::string>(uuidImg);
        writer->Write(image.value());
      }
    }
  }
}

boost::uuids::uuid CorePbImage::addData(const seerep::pb::Image& img)
{
  seerep_core_msgs::DatasetIndexable dataForIndices = CorePbConversion::fromPb(img);

  auto hdf5io = getHdf5(dataForIndices.header.uuidProject);
  hdf5io->writeImage(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), img);

  m_seerepCore->addDataset(dataForIndices);

  return dataForIndices.header.uuidData;
}

void CorePbImage::getFileAccessorFromCore(boost::uuids::uuid project)
{
  auto hdf5file = m_seerepCore->getHdf5File(project);
  auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(project);
  auto imageIo = std::make_shared<seerep_hdf5_pb::Hdf5PbImage>(hdf5file, hdf5fileMutex);
  m_hdf5IoMap.insert(std::make_pair(project, imageIo));
}

std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> CorePbImage::getHdf5(boost::uuids::uuid project)
{
  // find the project based on its uuid
  auto hdf5io = m_hdf5IoMap.find(project);
  // if project was found add tf
  if (hdf5io != m_hdf5IoMap.end())
  {
    return hdf5io->second;
  }
  // if not found ask core
  else
  {
    // this throws an exeption if core has no project with the uuid
    getFileAccessorFromCore(project);
    // if getFileAccessorFromCore didn't throw an error, find project and return pointer
    return m_hdf5IoMap.find(project)->second;
  };
}

}  // namespace seerep_core_pb
