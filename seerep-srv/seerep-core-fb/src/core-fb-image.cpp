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

std::vector<flatbuffers::Offset<seerep::fb::Image>> CoreFbImage::getData(const seerep::fb::Query& query)
{
  std::cout << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore;
  boost::uuids::string_generator gen;

  for (auto projectuuid : *query.projectuuid())
  {
    queryCore.projects.push_back(gen(projectuuid->str()));
  }
  for (auto label : *query.label())
  {
    queryCore.label.push_back(label->str());
  }
  queryCore.timeinterval.timeMin.seconds = query.timeinterval()->time_min()->seconds();
  queryCore.timeinterval.timeMax.seconds = query.timeinterval()->time_max()->seconds();
  queryCore.timeinterval.timeMin.nanos = query.timeinterval()->time_min()->nanos();
  queryCore.timeinterval.timeMax.nanos = query.timeinterval()->time_max()->nanos();

  queryCore.header.frameId = query.boundingbox()->header()->frame_id()->str();
  queryCore.boundingbox.min_corner().set<0>(query.boundingbox()->point_min()->x());
  queryCore.boundingbox.min_corner().set<1>(query.boundingbox()->point_min()->y());
  queryCore.boundingbox.min_corner().set<2>(query.boundingbox()->point_min()->z());
  queryCore.boundingbox.max_corner().set<0>(query.boundingbox()->point_max()->x());
  queryCore.boundingbox.max_corner().set<1>(query.boundingbox()->point_max()->y());
  queryCore.boundingbox.max_corner().set<2>(query.boundingbox()->point_max()->z());

  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getImage(queryCore);

  std::vector<flatbuffers::Offset<seerep::fb::Image>> resultImages;
  for (auto project : resultCore.queryResultProjects)
  {
    for (auto uuidImg : project.dataUuids)
    {
      auto hdf5io = getHdf5(project.projectUuid);
      std::optional<flatbuffers::Offset<seerep::fb::Image>> image = hdf5io->readImage(
          boost::lexical_cast<std::string>(uuidImg), boost::lexical_cast<std::string>(project.projectUuid));
      if (image)
      {
        resultImages.push_back(image.value());
      }
    }
  }
  return resultImages;
}

boost::uuids::uuid CoreFbImage::addData(const seerep::fb::Image& img)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid;
  if (img.header()->uuid_msgs()->str().empty())
  {
    uuid = boost::uuids::random_generator()();
  }
  else
  {
    uuid = gen(img.header()->uuid_msgs()->str());
  }
  auto hdf5io = getHdf5(gen(img.header()->uuid_project()->str()));
  hdf5io->writeImage(boost::lexical_cast<std::string>(uuid), img);

  seerep_core_msgs::DatasetIndexable dataForIndices;
  dataForIndices.header.frameId = img.header()->frame_id()->str();
  dataForIndices.header.timestamp.seconds = img.header()->stamp()->seconds();
  dataForIndices.header.timestamp.nanos = img.header()->stamp()->nanos();
  dataForIndices.header.uuidData = uuid;
  dataForIndices.header.uuidProject = gen(img.header()->uuid_project()->str());
  // set bounding box for images to 0. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(0);
  dataForIndices.boundingbox.min_corner().set<1>(0);
  dataForIndices.boundingbox.min_corner().set<2>(0);
  dataForIndices.boundingbox.max_corner().set<0>(0);
  dataForIndices.boundingbox.max_corner().set<1>(0);
  dataForIndices.boundingbox.max_corner().set<2>(0);

  // semantic
  dataForIndices.labels.reserve(img.labels_general()->size() + img.labels_bb()->size());
  for (auto label : *img.labels_general())
  {
    dataForIndices.labels.push_back(label->str());
  }

  for (auto label : *img.labels_bb())
  {
    dataForIndices.labels.push_back(label->label()->str());
  }

  m_seerepCore->addImage(dataForIndices);

  return uuid;
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
