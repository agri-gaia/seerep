#include "seerep-core-pb/core-pb-image.h"

namespace seerep_core_pb
{
CorePbImage::CorePbImage(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : m_seerepCore->getProjects())
  {
    auto hdf5file = m_seerepCore->getHdf5File(projectInfo.uuid);
    auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(projectInfo.uuid);
    auto imageIo = std::make_shared<seerep_io_pb::IoPbImage>(hdf5file, hdf5fileMutex);

    m_hdf5IoMap.insert(std::make_pair(projectInfo.uuid, imageIo));
  }
}

CorePbImage::~CorePbImage()
{
}

std::vector<seerep::Image> CorePbImage::getData(const seerep::Query& query)
{
  std::cout << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore;
  // TODO do the transform
  boost::uuids::string_generator gen;
  queryCore.projects.push_back(gen(query.projectuuid()));
  for (auto label : query.label())
  {
    queryCore.label.push_back(label);
  }
  queryCore.timeinterval.timeMin.seconds = query.timeinterval().time_min();
  queryCore.timeinterval.timeMax.seconds = query.timeinterval().time_max();
  queryCore.timeinterval.timeMin.nanos = 0;
  queryCore.timeinterval.timeMax.nanos = 0;

  queryCore.header.frameId = query.boundingbox().header().frame_id();
  queryCore.boundingbox.min_corner().set<0>(query.boundingbox().point_min().x());
  queryCore.boundingbox.min_corner().set<1>(query.boundingbox().point_min().y());
  queryCore.boundingbox.min_corner().set<2>(query.boundingbox().point_min().z());
  queryCore.boundingbox.max_corner().set<0>(query.boundingbox().point_max().x());
  queryCore.boundingbox.max_corner().set<1>(query.boundingbox().point_max().y());
  queryCore.boundingbox.max_corner().set<2>(query.boundingbox().point_max().z());

  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getImage(queryCore);

  std::vector<seerep::Image> resultImages;
  for (auto project : resultCore.queryResultProjects)
  {
    for (auto uuidImg : project.dataUuids)
    {
      std::optional<seerep::Image> image =
          m_hdf5IoMap.at(project.projectUuid)->readImage(boost::lexical_cast<std::string>(uuidImg));
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
  // TODO check if project uuid is valid
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid;
  if (img.header().uuid_msgs().empty())
  {
    uuid = boost::uuids::random_generator()();
  }
  else
  {
    uuid = gen(img.header().uuid_msgs());
  }

  m_hdf5IoMap.at(gen(img.header().uuid_project()))->writeImage(boost::lexical_cast<std::string>(uuid), img);

  seerep_core_msgs::DatasetIndexable dataForIndices;
  dataForIndices.header.frameId = img.header().frame_id();
  dataForIndices.header.timestamp.seconds = img.header().stamp().seconds();
  dataForIndices.header.timestamp.nanos = img.header().stamp().nanos();
  dataForIndices.header.uuidData = uuid;
  dataForIndices.header.uuidProject = gen(img.header().uuid_project());
  // set bounding box for images to 0. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(0);
  dataForIndices.boundingbox.min_corner().set<1>(0);
  dataForIndices.boundingbox.min_corner().set<2>(0);
  dataForIndices.boundingbox.max_corner().set<0>(0);
  dataForIndices.boundingbox.max_corner().set<1>(0);
  dataForIndices.boundingbox.max_corner().set<2>(0);

  // semantic
  dataForIndices.labels.reserve(img.labels_general().size() + img.labels_bb().size());
  for (auto label : img.labels_general())
  {
    dataForIndices.labels.push_back(label);
  }

  for (auto label : img.labels_bb())
  {
    dataForIndices.labels.push_back(label.label());
  }

  m_seerepCore->addImage(dataForIndices);

  return uuid;
}

}  // namespace seerep_core_pb
