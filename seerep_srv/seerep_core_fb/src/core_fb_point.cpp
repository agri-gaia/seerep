#include "seerep_core_fb/core_fb_point.h"

namespace seerep_core_fb
{
CoreFbPoint::CoreFbPoint(std::shared_ptr<seerep_core::Core> seerepCore)
  : m_seerepCore(seerepCore)
{
  CoreFbGeneral::getAllFileAccessorFromCore(m_seerepCore, m_hdf5IoMap);
}

CoreFbPoint::~CoreFbPoint()
{
}

void CoreFbPoint::getData(
    const seerep::fb::Query* query,
    grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointStamped>>* const
        writer)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore = seerep_core_fb::CoreFbConversion::fromFb(
      query, seerep_core_msgs::Datatype::Point);

  seerep_core_msgs::QueryResult resultCore =
      m_seerepCore->getDataset(queryCore);

  for (auto project : resultCore.queryResultProjects)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "sending images from project"
        << boost::lexical_cast<std::string>(project.projectUuid);
    for (auto uuidPoint : project.dataOrInstanceUuids)
    {
      auto point =
          CoreFbGeneral::getHdf5(project.projectUuid, m_seerepCore, m_hdf5IoMap)
              ->readPoint(boost::lexical_cast<std::string>(uuidPoint));

      if (point)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
            << "sending point " << boost::lexical_cast<std::string>(uuidPoint);
        writer->Write(point.value());
      }
    }
  }
}

boost::uuids::uuid
CoreFbPoint::addDataToHdf5(const seerep::fb::PointStamped* point)
{
  seerep_core_msgs::DatasetIndexable dataForIndices =
      CoreFbConversion::fromFb(point);

  auto hdf5io = CoreFbGeneral::getHdf5(dataForIndices.header.uuidProject,
                                       m_seerepCore, m_hdf5IoMap);
  hdf5io->writePoint(
      boost::lexical_cast<std::string>(dataForIndices.header.uuidData), point);

  return dataForIndices.header.uuidData;
}

void CoreFbPoint::buildIndices(
    const std::vector<std::pair<std::string, boost::uuids::uuid>>&
        projectPointUuids)
{
  for (auto [projectUuid, pointUuid] : projectPointUuids)
  {
    auto point = CoreFbGeneral::getHdf5(projectUuid, m_seerepCore, m_hdf5IoMap)
                     ->readPoint(boost::lexical_cast<std::string>(pointUuid));

    if (!point.has_value())
    {
      throw std::runtime_error("points couldn't be retrieved correctly!");
    }
    seerep_core_msgs::DatasetIndexable dataForIndices =
        CoreFbConversion::fromFb(point.value().GetRoot());

    m_seerepCore->addDataset(dataForIndices);
  }
}

void CoreFbPoint::addAttributes(
    const seerep::fb::AttributesStamped& attributesStamped)
{
  auto hdf5io =
      CoreFbGeneral::getHdf5(attributesStamped.header()->uuid_project()->str(),
                             m_seerepCore, m_hdf5IoMap);
  hdf5io->writeAdditionalPointAttributes(attributesStamped);
}
}  // namespace seerep_core_fb
