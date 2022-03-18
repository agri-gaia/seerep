#include "seerep-core-pb/core-pb-pointcloud.h"

namespace seerep_core_pb
{
CorePbPointCloud::CorePbPointCloud(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : m_seerepCore->getProjects())
  {
    auto hdf5file = m_seerepCore->getHdf5File(projectInfo.uuid);
    auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(projectInfo.uuid);
    auto pointCloudIo = std::make_shared<seerep_hdf5_pb::Hdf5PbPointCloud>(hdf5file, hdf5fileMutex);

    m_hdf5IoMap.insert(std::make_pair(projectInfo.uuid, pointCloudIo));
  }
}

CorePbPointCloud::~CorePbPointCloud()
{
}

std::vector<seerep::PointCloud2> CorePbPointCloud::getData(const seerep::Query& query)
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

  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getPointCloud(queryCore);

  std::vector<seerep::PointCloud2> resultPointClouds;
  for (auto project : resultCore.queryResultProjects)
  {
    for (auto uuidPc : project.dataUuids)
    {
      auto hdf5io = getHdf5(project.projectUuid);
      std::optional<seerep::PointCloud2> pc = hdf5io->readPointCloud2(boost::lexical_cast<std::string>(uuidPc));
      if (pc)
      {
        resultPointClouds.push_back(pc.value());
      }
    }
  }
  return resultPointClouds;
}

boost::uuids::uuid CorePbPointCloud::addData(const seerep::PointCloud2& pc)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid;
  if (pc.header().uuid_msgs().empty())
  {
    uuid = boost::uuids::random_generator()();
  }
  else
  {
    uuid = gen(pc.header().uuid_msgs());
  }
  auto hdf5io = getHdf5(gen(pc.header().uuid_project()));
  hdf5io->writePointCloud2(boost::lexical_cast<std::string>(uuid), pc);

  seerep_core_msgs::DatasetIndexable dataForIndices;
  dataForIndices.header.frameId = pc.header().frame_id();
  dataForIndices.header.timestamp.seconds = pc.header().stamp().seconds();
  dataForIndices.header.timestamp.nanos = pc.header().stamp().nanos();
  dataForIndices.header.uuidData = uuid;
  dataForIndices.header.uuidProject = gen(pc.header().uuid_project());

  // TODO load from file
  std::vector<float> bb = hdf5io->loadBoundingBox(boost::lexical_cast<std::string>(uuid));
  dataForIndices.boundingbox.min_corner().set<0>(bb.at(0));
  dataForIndices.boundingbox.min_corner().set<1>(bb.at(1));
  dataForIndices.boundingbox.min_corner().set<2>(bb.at(2));
  dataForIndices.boundingbox.max_corner().set<0>(bb.at(3));
  dataForIndices.boundingbox.max_corner().set<1>(bb.at(4));
  dataForIndices.boundingbox.max_corner().set<2>(bb.at(5));

  // semantic
  dataForIndices.labels.reserve(pc.labels_general().size() + pc.labels_bb().size());
  for (auto label : pc.labels_general())
  {
    dataForIndices.labels.push_back(label);
  }

  for (auto label : pc.labels_bb())
  {
    dataForIndices.labels.push_back(label.label());
  }

  m_seerepCore->addImage(dataForIndices);

  return uuid;
}

void CorePbPointCloud::getFileAccessorFromCore(boost::uuids::uuid project)
{
  auto hdf5file = m_seerepCore->getHdf5File(project);
  auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(project);
  auto pointCloudIo = std::make_shared<seerep_hdf5_pb::Hdf5PbPointCloud>(hdf5file, hdf5fileMutex);
  m_hdf5IoMap.insert(std::make_pair(project, pointCloudIo));
}

std::shared_ptr<seerep_hdf5_pb::Hdf5PbPointCloud> CorePbPointCloud::getHdf5(boost::uuids::uuid project)
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
