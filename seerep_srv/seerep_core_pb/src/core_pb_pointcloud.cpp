#include "seerep_core_pb/core_pb_pointcloud.h"

namespace seerep_core_pb
{
CorePbPointCloud::CorePbPointCloud(std::shared_ptr<seerep_core::Core> seerepCore)
  : m_seerepCore(seerepCore)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : m_seerepCore->getProjects())
  {
    auto hdf5file = m_seerepCore->getHdf5File(projectInfo.uuid);
    auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(projectInfo.uuid);
    auto pointCloudIo = std::make_shared<seerep_hdf5_pb::Hdf5PbPointCloud>(
        hdf5file, hdf5fileMutex);

    m_hdf5IoMap.insert(std::make_pair(projectInfo.uuid, pointCloudIo));
  }
}

CorePbPointCloud::~CorePbPointCloud()
{
}

std::vector<seerep::pb::PointCloud2>
CorePbPointCloud::getData(const seerep::pb::Query& query)
{
  std::cout << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore =
      CorePbConversion::fromPb(query, seerep_core_msgs::Datatype::PointCloud);
  seerep_core_msgs::QueryResult resultCore =
      m_seerepCore->getDataset(queryCore);

  std::vector<seerep::pb::PointCloud2> resultPointClouds;
  for (auto project : resultCore.queryResultProjects)
  {
    for (auto uuidPc : project.dataOrInstanceUuids)
    {
      auto hdf5io = getHdf5(project.projectUuid);
      std::optional<seerep::pb::PointCloud2> pc =
          hdf5io->readPointCloud2(boost::lexical_cast<std::string>(uuidPc));
      if (pc)
      {
        resultPointClouds.push_back(pc.value());
      }
    }
  }
  return resultPointClouds;
}

boost::uuids::uuid CorePbPointCloud::addData(const seerep::pb::PointCloud2& pc)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid messageuuid;

  // generate uuids if not provided in the message
  if (pc.header().uuid_msgs().empty())
  {
    messageuuid = boost::uuids::random_generator()();
  }
  else
  {
    messageuuid = gen(pc.header().uuid_msgs());
  }

  auto hdf5io = getHdf5(gen(pc.header().uuid_project()));
  hdf5io->writePointCloud2(boost::lexical_cast<std::string>(messageuuid), pc);

  seerep_core_msgs::DatasetIndexable dataForIndices;
  dataForIndices.header.datatype = seerep_core_msgs::Datatype::PointCloud;
  dataForIndices.header.frameId = pc.header().frame_id();
  dataForIndices.header.timestamp.seconds = pc.header().stamp().seconds();
  dataForIndices.header.timestamp.nanos = pc.header().stamp().nanos();
  dataForIndices.header.uuidData = messageuuid;
  dataForIndices.header.uuidProject = gen(pc.header().uuid_project());

  std::vector<float> bb =
      hdf5io->loadBoundingBox(boost::lexical_cast<std::string>(messageuuid));
  dataForIndices.boundingbox.min_corner().set<0>(bb.at(0));
  dataForIndices.boundingbox.min_corner().set<1>(bb.at(1));
  dataForIndices.boundingbox.min_corner().set<2>(bb.at(2));
  dataForIndices.boundingbox.max_corner().set<0>(bb.at(3));
  dataForIndices.boundingbox.max_corner().set<1>(bb.at(4));
  dataForIndices.boundingbox.max_corner().set<2>(bb.at(5));

  // semantic

  if (!pc.labels().empty())
  {
    for (auto labelsCategories : pc.labels())
    {
      std::vector<seerep_core_msgs::Label> labelVector;
      if (!labelsCategories.labels().empty())
      {
        for (auto label : labelsCategories.labels())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.instanceuuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelVector.push_back(seerep_core_msgs::Label{
              .label = label.label(),
              .labelIdDatumaro = label.labeliddatumaro(),
              .uuidInstance = uuidInstance,
              .instanceIdDatumaro = label.instanceiddatumaro() });
        }
        dataForIndices.labelsCategory.emplace(
            labelsCategories.category().c_str(),
            seerep_core_msgs::LabelDatumaro{
                .labels = labelVector,
                .datumaroJson = labelsCategories.datumarojson() });
      }
    }
  }

  m_seerepCore->addDataset(dataForIndices);

  return messageuuid;
}

void CorePbPointCloud::getFileAccessorFromCore(boost::uuids::uuid project)
{
  auto hdf5file = m_seerepCore->getHdf5File(project);
  auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(project);
  auto pointCloudIo = std::make_shared<seerep_hdf5_pb::Hdf5PbPointCloud>(
      hdf5file, hdf5fileMutex);
  m_hdf5IoMap.insert(std::make_pair(project, pointCloudIo));
}

std::shared_ptr<seerep_hdf5_pb::Hdf5PbPointCloud>
CorePbPointCloud::getHdf5(boost::uuids::uuid project)
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
