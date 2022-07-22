#include "seerep-core-fb/core-fb-image.h"

namespace seerep_core_fb
{
CoreFbImage::CoreFbImage(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : m_seerepCore->getProjects())
  {
    getFileAccessorFromCore(projectInfo.uuid);
  }
}

CoreFbImage::~CoreFbImage()
{
}

void CoreFbImage::getData(const seerep::fb::Query& query,
                          grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* const writer)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore = seerep_core_fb::CoreFbConversion::fromFb(query);

  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getDataset(queryCore);

  for (auto project : resultCore.queryResultProjects)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "sending images from project" << boost::lexical_cast<std::string>(project.projectUuid);
    for (auto uuidImg : project.dataOrInstanceUuids)
    {
      auto img = getHdf5(project.projectUuid)->readImage(boost::lexical_cast<std::string>(uuidImg));

      if (img)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
            << "sending img " << boost::lexical_cast<std::string>(uuidImg);
        writer->Write(img.value());
      }
    }
  }
}

boost::uuids::uuid CoreFbImage::addData(const seerep::fb::Image& img)
{
  seerep_core_msgs::DatasetIndexable dataForIndices = CoreFbConversion::fromFb(img);

  auto hdf5io = getHdf5(dataForIndices.header.uuidProject);
  hdf5io->writeImage(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), img);

  m_seerepCore->addDataset(dataForIndices);

  return dataForIndices.header.uuidData;
}

void CoreFbImage::addBoundingBoxesLabeled(const seerep::fb::BoundingBoxes2DLabeledStamped& boundingBoxes2dlabeled)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuidMsg = gen(boundingBoxes2dlabeled.header()->uuid_msgs()->str());
  boost::uuids::uuid uuidProject = gen(boundingBoxes2dlabeled.header()->uuid_project()->str());

  auto hdf5io = getHdf5(uuidProject);
  hdf5io->writeImageBoundingBox2DLabeled(boost::lexical_cast<std::string>(uuidMsg), boundingBoxes2dlabeled);

  std::vector<std::string> labels;
  for (auto bb : *boundingBoxes2dlabeled.labels_bb())
  {
    labels.push_back(bb->labelWithInstance()->label()->str());
  }
  // this only adds labels to the image in the core
  // if there are already bounding box labels for this image
  // those labels must be removed separately. The hdfio currently overrides
  // existing labels. The data is only correct if labels are added and there
  // weren't any bounding box labels before

  /// @todo also add instance
  m_seerepCore->addLabels(seerep_core_msgs::Datatype::Images, labels, uuidMsg, uuidProject);
}

void CoreFbImage::getFileAccessorFromCore(boost::uuids::uuid project)
{
  auto hdf5file = m_seerepCore->getHdf5File(project);
  auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(project);
  auto imageIo = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(hdf5file, hdf5fileMutex);
  m_hdf5IoMap.insert(std::make_pair(project, imageIo));
}

std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> CoreFbImage::getHdf5(boost::uuids::uuid project)
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

}  // namespace seerep_core_fb
