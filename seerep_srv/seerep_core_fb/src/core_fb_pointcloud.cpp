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

boost::uuids::uuid CoreFbPointCloud::addDataToHdf5(const seerep::fb::PointCloud2& pcl)
{
  seerep_core_msgs::DatasetIndexable dataForIndices = CoreFbConversion::fromFb(pcl);

  auto hdf5io = CoreFbGeneral::getHdf5(dataForIndices.header.uuidProject, m_seerepCore, m_hdf5IoMap);
  auto [min_corner, max_corner] = hdf5io->computeBoundingBox(pcl);
  hdf5io->writePointCloud2(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), pcl);
  hdf5io->writeBoundingBox(boost::lexical_cast<std::string>(dataForIndices.header.uuidData), min_corner, max_corner);

  return dataForIndices.header.uuidData;
}

void CoreFbPointCloud::buildIndices(std::vector<std::pair<std::string, boost::uuids::uuid>>& projectPclUuids)
{
  seerep_core_msgs::AABB bb;
  for (auto [projectUuid, pclUuid] : projectPclUuids)
  {
    auto hdf5io = CoreFbGeneral::getHdf5(projectUuid, m_seerepCore, m_hdf5IoMap);
    auto uuidStr = boost::lexical_cast<std::string>(pclUuid);

    auto optionalPcl = hdf5io->readPointCloud2(uuidStr, true);
    hdf5io->readAABB(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, uuidStr, bb);

    if (!optionalPcl.has_value())
    {
      throw std::runtime_error("pointclouds couldn't be retrieved correctly!");
    }

    auto pcl = optionalPcl.value().GetRoot();
    auto dataForIndices = CoreFbConversion::fromFb(*pcl);

    dataForIndices.boundingbox.max_corner() = bb.max_corner();
    dataForIndices.boundingbox.min_corner() = bb.min_corner();

    m_seerepCore->addDataset(dataForIndices);
  }
}

void CoreFbPointCloud::addLabel(const seerep::fb::DatasetUuidLabel& datasetUuidLabel)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuidMsg = gen(datasetUuidLabel.datasetUuid()->str());
  boost::uuids::uuid uuidProject = gen(datasetUuidLabel.projectUuid()->str());

  auto hdf5io = CoreFbGeneral::getHdf5(uuidProject, m_seerepCore, m_hdf5IoMap);

  hdf5io->writeLabelsFb(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD,
                        datasetUuidLabel.datasetUuid()->str(), datasetUuidLabel.labels());

  auto labelPerCategory = seerep_core_fb::CoreFbGeneral::extractLabelsPerCategory(datasetUuidLabel);

  m_seerepCore->addLabels(seerep_core_msgs::Datatype::PointCloud, labelPerCategory, uuidMsg, uuidProject);
}

} /* namespace seerep_core_fb */
