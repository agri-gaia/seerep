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
      std::optional<seerep::PointCloud2> pc =
          m_hdf5IoMap.at(project.projectUuid)->readPointCloud2(boost::lexical_cast<std::string>(uuidPc));
      if (pc)
      {
        resultPointClouds.push_back(pc.value());
      }
    }
  }
  return resultPointClouds;
}

}  // namespace seerep_core_pb
