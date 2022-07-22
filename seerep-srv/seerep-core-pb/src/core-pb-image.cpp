#include "seerep-core-pb/core-pb-image.h"

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

std::vector<seerep::Image> CorePbImage::getData(const seerep::Query& query)
{
  std::cout << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore = CorePbConversion::fromPb(query, seerep_core_msgs::Datatype::Images);

  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getDataset(queryCore);

  std::vector<seerep::Image> resultImages;
  for (auto project : resultCore.queryResultProjects)
  {
    for (auto uuidImg : project.dataOrInstanceUuids)
    {
      auto hdf5io = getHdf5(project.projectUuid);
      std::optional<seerep::Image> image = hdf5io->readImage(boost::lexical_cast<std::string>(uuidImg));
      if (image)
      {
        resultImages.push_back(image.value());
      }
    }
  }
  return resultImages;
}

boost::uuids::uuid CorePbImage::addData(const seerep::Image& img)
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
