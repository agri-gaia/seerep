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
  seerep_core_msgs::Query queryCore = CorePbConversion::fromPb(query, seerep_core_msgs::Datatype::PointCloud);
  seerep_core_msgs::QueryResult resultCore = m_seerepCore->getDataset(queryCore);

  std::vector<seerep::PointCloud2> resultPointClouds;
  for (auto project : resultCore.queryResultProjects)
  {
    for (auto uuidPc : project.dataOrInstanceUuids)
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

  std::vector<float> bb = hdf5io->loadBoundingBox(boost::lexical_cast<std::string>(messageuuid));
  dataForIndices.boundingbox.min_corner().set<0>(bb.at(0));
  dataForIndices.boundingbox.min_corner().set<1>(bb.at(1));
  dataForIndices.boundingbox.min_corner().set<2>(bb.at(2));
  dataForIndices.boundingbox.max_corner().set<0>(bb.at(3));
  dataForIndices.boundingbox.max_corner().set<1>(bb.at(4));
  dataForIndices.boundingbox.max_corner().set<2>(bb.at(5));

  // semantic
  int labelSizeAll = 0;
  if (!pc.general_labels().empty())
  {
    labelSizeAll += pc.general_labels().size();
  }
  if (!pc.bounding_box_labels().empty())
  {
    labelSizeAll += pc.bounding_box_labels().size();
  }

  if (!pc.general_labels().empty())
  {
    for (auto labelsCategories : pc.general_labels())
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      if (!labelsCategories.label_with_instances().empty())
      {
        for (auto label : labelsCategories.label_with_instances())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.instance_uuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelWithInstanceVector.push_back(
              seerep_core_msgs::LabelWithInstance{ .label = label.label(), .uuidInstance = uuidInstance });
        }
        dataForIndices.labelsWithInstancesWithCategory.emplace(labelsCategories.category().c_str(),
                                                               labelWithInstanceVector);
      }
    }
  }

  if (!pc.bounding_box_labels().empty())
  {
    for (auto labelsCategories : pc.bounding_box_labels())
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      if (!labelsCategories.labeled_bounding_boxes().empty())
      {
        for (auto label : labelsCategories.labeled_bounding_boxes())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.label_with_instance().instance_uuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelWithInstanceVector.push_back(seerep_core_msgs::LabelWithInstance{
              .label = label.label_with_instance().label(), .uuidInstance = uuidInstance });
        }
      }
      dataForIndices.labelsWithInstancesWithCategory.emplace(labelsCategories.category().c_str(),
                                                             labelWithInstanceVector);
    }
  }

  m_seerepCore->addDataset(dataForIndices);

  return messageuuid;
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
