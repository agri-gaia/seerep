#include "seerep-core-fb/core-fb-image.h"

namespace seerep_core_fb
{
CoreFbImage::CoreFbImage(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  CoreFbGeneral::getAllFileAccessorFromCore(m_seerepCore, m_hdf5IoMap);
}

CoreFbImage::~CoreFbImage()
{
}

void CoreFbImage::getData(const seerep::fb::Query* query,
                          grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* const writer)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore =
      seerep_core_fb::CoreFbConversion::fromFb(query, seerep_core_msgs::Datatype::Image);

  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getDataset(queryCore);

  for (auto project : resultCore.queryResultProjects)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "sending images from project" << boost::lexical_cast<std::string>(project.projectUuid);
    for (auto uuidImg : project.dataOrInstanceUuids)
    {
      auto img = CoreFbGeneral::getHdf5(project.projectUuid, m_seerepCore, m_hdf5IoMap)
                     ->readImage(boost::lexical_cast<std::string>(uuidImg), queryCore.withoutData);

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

  auto hdf5io = CoreFbGeneral::getHdf5(dataForIndices.header.uuidProject, m_seerepCore, m_hdf5IoMap);
  hdf5io->writeImage(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), img);

  m_seerepCore->addDataset(dataForIndices);

  return dataForIndices.header.uuidData;
}

void CoreFbImage::addBoundingBoxesLabeled(const seerep::fb::BoundingBoxes2DLabeledStamped& boundingBoxes2dlabeled)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuidMsg = gen(boundingBoxes2dlabeled.header()->uuid_msgs()->str());
  boost::uuids::uuid uuidProject = gen(boundingBoxes2dlabeled.header()->uuid_project()->str());

  auto hdf5io = CoreFbGeneral::getHdf5(uuidProject, m_seerepCore, m_hdf5IoMap);

  hdf5io->writeImageBoundingBox2DLabeled(boost::lexical_cast<std::string>(uuidMsg), boundingBoxes2dlabeled.labels_bb());

  std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>> labelWithInstancePerCategory;
  for (auto bbWithCategory : *boundingBoxes2dlabeled.labels_bb())
  {
    if (bbWithCategory->boundingBox2dLabeled())
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelsWithInstances;
      for (auto bb : *bbWithCategory->boundingBox2dLabeled())
      {
        seerep_core_msgs::LabelWithInstance labelWithInstance;
        labelWithInstance.label = bb->labelWithInstance()->label()->str();

        try
        {
          labelWithInstance.uuidInstance = gen(bb->labelWithInstance()->instanceUuid()->str());
        }
        catch (std::runtime_error const& e)
        {
          labelWithInstance.uuidInstance = boost::uuids::nil_uuid();
        }
        labelsWithInstances.push_back(labelWithInstance);
      }
      labelWithInstancePerCategory.emplace(bbWithCategory->category()->c_str(), labelsWithInstances);
    }
    // this only adds labels to the image in the core
    // if there are already bounding box labels for this image
    // those labels must be removed separately. The hdfio currently overrides
    // existing labels. The data is only correct if labels are added and there
    // weren't any bounding box labels before

    m_seerepCore->addLabels(seerep_core_msgs::Datatype::Image, labelWithInstancePerCategory, uuidMsg, uuidProject);
  }
}

}  // namespace seerep_core_fb
