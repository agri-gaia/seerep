#include "seerep-core-fb/core-fb-point.h"

namespace seerep_core_fb
{
CoreFbPoint::CoreFbPoint(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  CoreFbGeneral::getAllFileAccessorFromCore(m_seerepCore, m_hdf5IoMap);
}

CoreFbPoint::~CoreFbPoint()
{
}

void CoreFbPoint::getData(const seerep::fb::Query& query,
                          grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointStamped>>* const writer)
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
      auto point = CoreFbGeneral::getHdf5(project.projectUuid, m_seerepCore, m_hdf5IoMap)
                       ->readPoint(boost::lexical_cast<std::string>(uuidImg));

      if (point)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
            << "sending point " << boost::lexical_cast<std::string>(uuidImg);
        writer->Write(point.value());
      }
    }
  }
}

boost::uuids::uuid CoreFbPoint::addData(const seerep::fb::PointStamped& point)
{
  seerep_core_msgs::DatasetIndexable dataForIndices = CoreFbConversion::fromFb(point);

  auto hdf5io = CoreFbGeneral::getHdf5(dataForIndices.header.uuidProject, m_seerepCore, m_hdf5IoMap);
  hdf5io->writePoint(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), point);

  m_seerepCore->addDataset(dataForIndices);

  return dataForIndices.header.uuidData;
}

}  // namespace seerep_core_fb
