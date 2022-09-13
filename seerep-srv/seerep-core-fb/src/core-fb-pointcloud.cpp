#include "seerep-core-fb/core-fb-pointcloud.h"

namespace seerep_core_fb
{
CoreFbPointCloud::CoreFbPointCloud(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  CoreFbGeneral::getAllFileAccessorFromCore(m_seerepCore, m_hdf5IoMap);
}

CoreFbPointCloud::~CoreFbPointCloud()
{
}

std::vector<seerep::fb::PointCloud2> CoreFbPointCloud::getData(const seerep::fb::Query& query)
{
  // BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading point cloud from pointclouds/";
  // seerep_core_msgs::Query queryCore = CoreFbConversion::fromFb(query, seerep_core_msgs::Datatype::PointCloud);
  // seerep_core_msgs::QueryResult resultCore = m_seerepCore->getDataset(queryCore);

  // std::vector<seerep::fb::PointCloud2> resultPointClouds;
  // for (auto project : resultCore.queryResultProjects)
  // {
  //   BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
  //       << "sending point cloud from project" << boost::lexical_cast<std::string>(project.projectUuid);
  //   for (auto uuidPc : project.dataOrInstanceUuids)
  //   {
  //     auto img = CoreFbGeneral::getHdf5(project.projectUuid, m_seerepCore, m_hdf5IoMap)
  //                    ->readPointCloud(boost::lexical_cast<std::string>(uuidImg), queryCore.withoutData);
  //     if (pc)
  //     {
  //       BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
  //           << "sending point cloud " << boost::lexical_cast<std::string>(uuidImg);
  //       resultPointClouds.push_back(pc.value());
  //     }
  //   }
  // }
  // return resultPointClouds;
}

boost::uuids::uuid CoreFbPointCloud::addData(const seerep::fb::PointCloud2& pc)
{
  seerep_core_msgs::DatasetIndexable dataForIndices = CoreFbConversion::fromFb(pc);

  auto hdf5io = CoreFbGeneral::getHdf5(dataForIndices.header.uuidProject, m_seerepCore, m_hdf5IoMap);
  hdf5io->writePointCloud2(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), &pc);

  m_seerepCore->addDataset(dataForIndices);

  return dataForIndices.header.uuidData;
}

} /* namespace seerep_core_fb */
