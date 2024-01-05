#include "seerep_core_fb/core_fb_image.h"

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

boost::uuids::uuid CoreFbImage::addDataToHdf5(const seerep::fb::Image& img)
{
  seerep_core_msgs::DatasetIndexable dataForIndices = CoreFbConversion::fromFb(img);

  boost::uuids::string_generator gen;

  seerep_core_msgs::camera_intrinsics_query camintrinsics_query;
  camintrinsics_query.uuidProject = gen(img.header()->uuid_project()->str());
  camintrinsics_query.uuidCameraIntrinsics = gen(img.uuid_cameraintrinsics()->str());

  if (m_seerepCore->cameraIntrinsicExists(camintrinsics_query))
  {
    auto hdf5io = CoreFbGeneral::getHdf5(dataForIndices.header.uuidProject, m_seerepCore, m_hdf5IoMap);
    hdf5io->writeImage(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), img);

    return dataForIndices.header.uuidData;
  }
  else
  {
    throw std::invalid_argument("Invalid uuid for camera intrinsics or project recieved");
  }
}

void CoreFbImage::buildIndices(const std::vector<std::pair<std::string, boost::uuids::uuid>>& projecImgUuids)
{
  for (auto [projectUuid, imgUuid] : projecImgUuids)
  {
    auto hdf5io = CoreFbGeneral::getHdf5(projectUuid, m_seerepCore, m_hdf5IoMap);

    auto optionalImg = hdf5io->readImage(boost::lexical_cast<std::string>(imgUuid), true);

    if (!optionalImg.has_value())
    {
      throw std::runtime_error("images couldn't be retrieved correctly!");
    }

    auto img = optionalImg.value().GetRoot();
    auto dataForIndices = CoreFbConversion::fromFb(*img);

    hdf5io->computeFrustumBB(img->uuid_cameraintrinsics()->str(), dataForIndices.boundingbox);
    m_seerepCore->addDataset(dataForIndices);
  }
}

void CoreFbImage::addLabel(const seerep::fb::DatasetUuidLabel& datasetUuidLabel)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuidMsg = gen(datasetUuidLabel.datasetUuid()->str());
  boost::uuids::uuid uuidProject = gen(datasetUuidLabel.projectUuid()->str());

  auto hdf5io = CoreFbGeneral::getHdf5(uuidProject, m_seerepCore, m_hdf5IoMap);

  hdf5io->writeLabelsFb(boost::lexical_cast<std::string>(uuidMsg), datasetUuidLabel.datasetUuid()->str(),
                        datasetUuidLabel.labels());

  std::unordered_map<std::string, std::vector<seerep_core_msgs::Label>> labelPerCategory;
  for (auto labelCategory : *datasetUuidLabel.labels())
  {
    if (labelCategory->labels())
    {
      std::vector<seerep_core_msgs::Label> labelVector;
      for (auto labelFb : *labelCategory->labels())
      {
        seerep_core_msgs::Label label;
        label.label = labelFb->label()->str();
        label.labelIdDatumaro = labelFb->labelIdDatumaro();
        label.instanceIdDatumaro = labelFb->instanceIdDatumaro();

        try
        {
          label.uuidInstance = gen(labelFb->instanceUuid()->str());
        }
        catch (std::runtime_error const& e)
        {
          label.uuidInstance = boost::uuids::nil_uuid();
        }
        labelVector.push_back(label);
      }
      labelPerCategory.emplace(labelCategory->category()->c_str(), labelVector);
    }
    // this only adds labels to the image in the core
    // if there are already bounding box labels for this image
    // those labels must be removed separately. The hdfio currently overrides
    // existing labels. The data is only correct if labels are added and there
    // weren't any bounding box labels before

    m_seerepCore->addLabels(seerep_core_msgs::Datatype::Image, labelPerCategory, uuidMsg, uuidProject);
  }
}

}  // namespace seerep_core_fb
