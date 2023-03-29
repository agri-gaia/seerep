#include "seerep_core_fb/core_fb_pointcloud.h"

namespace seerep_core_fb
{
CoreFbPointCloud::CoreFbPointCloud(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  CoreFbGeneral::getAllFileAccessorFromCore(m_seerepCore, m_hdf5IoMap);
}

void CoreFbPointCloud::getData(const seerep::fb::Query* query,
                               grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* const writer)
{
  seerep_core_msgs::Query queryCore =
      seerep_core_fb::CoreFbConversion::fromFb(query, seerep_core_msgs::Datatype::PointCloud);

  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getDataset(queryCore);

  for (auto project : resultCore.queryResultProjects)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "sending flatbuffer point cloud from project: " << boost::lexical_cast<std::string>(project.projectUuid);

    for (auto uuidPc : project.dataOrInstanceUuids)
    {
      auto pc = CoreFbGeneral::getHdf5(project.projectUuid, m_seerepCore, m_hdf5IoMap)
                    ->readPointCloud2(boost::lexical_cast<std::string>(uuidPc), queryCore.withoutData);
      if (pc)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
            << "sending flatbuffers point cloud with UUID: " << boost::lexical_cast<std::string>(uuidPc);
        writer->Write(pc.value());
      }
    }
  }
}

// TODO refactor only a temporary solution
boost::uuids::uuid CoreFbPointCloud::addData(const seerep::fb::PointCloud2& pc)
{
  std::vector<float> boundingBox;

  seerep_core_msgs::DatasetIndexable dataForIndices = CoreFbConversion::fromFb(pc);

  auto hdf5io = CoreFbGeneral::getHdf5(dataForIndices.header.uuidProject, m_seerepCore, m_hdf5IoMap);
  hdf5io->writePointCloud2(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), pc, boundingBox);

  dataForIndices.boundingbox.min_corner().set<0>(boundingBox[0]);
  dataForIndices.boundingbox.min_corner().set<1>(boundingBox[1]);
  dataForIndices.boundingbox.min_corner().set<2>(boundingBox[2]);
  dataForIndices.boundingbox.max_corner().set<0>(boundingBox[3]);
  dataForIndices.boundingbox.max_corner().set<1>(boundingBox[4]);
  dataForIndices.boundingbox.max_corner().set<2>(boundingBox[5]);

  m_seerepCore->addDataset(dataForIndices);

  return dataForIndices.header.uuidData;
}

} /* namespace seerep_core_fb */
