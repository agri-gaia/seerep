#include "seerep-core/core.h"

namespace seerep_core
{
Core::Core(std::string dataFolder, bool loadHdf5Files) : m_dataFolder(dataFolder)
{
  if (loadHdf5Files)
  {
    recreateProjects();
  }
}

Core::~Core()
{
}

seerep_core_msgs::QueryResult Core::getDataset(const seerep_core_msgs::Query& query)
{
  // search all projects
  if (!query.projects)
  {
    return getDatasetFromAllProjects(query);
  }
  // Search only in project specified in query
  else
  {
    return getDatasetFromSpecificProjects(query);
  }
}

seerep_core_msgs::QueryResult Core::getInstances(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;

  // search all projects
  if (!query.projects)
  {
    for (auto& it : m_projects)
    {
      auto instances = it.second->getInstances(query);
      if (!instances.dataOrInstanceUuids.empty())
      {
        result.queryResultProjects.push_back(instances);
      }
    }
  }
  // Search only in project specified in query
  else
  {
    for (auto projectuuid : query.projects.value())
    {
      auto project = findProject(projectuuid);

      auto instances = project->second->getDataset(query);
      if (!instances.dataOrInstanceUuids.empty())
      {
        result.queryResultProjects.push_back(instances);
      }
    }
  }

  return result;
}

void Core::recreateProjects()
{
  for (const auto& entry : std::filesystem::directory_iterator(m_dataFolder))
  {
    if (entry.path().filename().extension() == ".h5")
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
          << "found " << entry.path().string() << " in HDF5 file.";

      try
      {
        boost::uuids::string_generator gen;
        boost::uuids::uuid uuid = gen(entry.path().filename().stem().string());

        auto project = std::make_shared<CoreProject>(uuid, entry.path().string());
        m_projects.insert(std::make_pair(uuid, project));
      }
      catch (const std::runtime_error& e)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
      }
    }
  }
}

void Core::createProject(const seerep_core_msgs::ProjectInfo& projectInfo)
{
  std::string filename = boost::lexical_cast<std::string>(projectInfo.uuid);
  std::string path = m_dataFolder + "/" + filename + ".h5";

  auto project = std::make_shared<CoreProject>(projectInfo.uuid, path, projectInfo.name, projectInfo.frameId);
  m_projects.insert(std::make_pair(projectInfo.uuid, project));
}

std::vector<seerep_core_msgs::ProjectInfo> Core::getProjects()
{
  std::vector<seerep_core_msgs::ProjectInfo> projectInfos;
  for (auto it = m_projects.begin(); it != m_projects.end(); ++it)
  {
    seerep_core_msgs::ProjectInfo projectinfo;
    projectinfo.name = it->second->getName();
    projectinfo.uuid = it->first;
    projectinfo.frameId = it->second->getFrameId();

    projectInfos.push_back(projectinfo);
  }

  return projectInfos;
}

void Core::addDataset(const seerep_core_msgs::DatasetIndexable& dataset)
{
  auto project = findProject(dataset.header.uuidProject);

  project->second->addDataset(dataset);
}

void Core::addLabels(const seerep_core_msgs::Datatype& datatype, std::vector<std::string>& labels,
                     const boost::uuids::uuid& msgUuid, const boost::uuids::uuid& projectuuid)
{
  auto project = findProject(projectuuid);

  project->second->addLabels(datatype, labels, msgUuid);
}

void Core::addTF(const geometry_msgs::TransformStamped& tf, const boost::uuids::uuid& projectuuid)
{
  auto project = findProject(projectuuid);
  project->second->addTF(tf);
}

std::optional<geometry_msgs::TransformStamped> Core::getTF(const seerep_core_msgs::QueryTf& transformQuery)
{
  try
  {
    auto project = findProject(transformQuery.project);
    return project->second->getTF(transformQuery);
  }
  catch (const std::runtime_error& e)
  {
    return std::nullopt;
  }
}

std::vector<std::string> Core::getFrames(const boost::uuids::uuid& projectuuid)
{
  try
  {
    auto project = findProject(projectuuid);
    return project->second->getFrames();
  }
  catch (const std::runtime_error& e)
  {
    return {};
  };
}

std::shared_ptr<std::mutex> Core::getHdf5FileMutex(const boost::uuids::uuid& projectuuid)
{
  try
  {
    auto project = findProject(projectuuid);
    return project->second->getHdf5FileMutex();
  }
  catch (const std::runtime_error& e)
  {
    return nullptr;
  }
}
std::shared_ptr<HighFive::File> Core::getHdf5File(const boost::uuids::uuid& projectuuid)
{
  try
  {
    auto project = findProject(projectuuid);
    return project->second->getHdf5File();
  }
  catch (const std::runtime_error& e)
  {
    return nullptr;
  }
}

std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::CoreProject>, boost::hash<boost::uuids::uuid>>::iterator
Core::findProject(const boost::uuids::uuid& projectuuid)
{
  auto project = m_projects.find(projectuuid);
  if (project != m_projects.end())
  {
    return project;
  }
  else
  {
    throw std::runtime_error("project " + boost::lexical_cast<std::string>(projectuuid) + "does not exist!");
  }
}

seerep_core_msgs::QueryResult Core::getDatasetFromAllProjects(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;
  for (auto& it : m_projects)
  {
    auto dataset = it.second->getDataset(query);
    addDatasetToResult(dataset, result);
  }
  return result;
}
seerep_core_msgs::QueryResult Core::getDatasetFromSpecificProjects(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;
  for (auto projectuuid : query.projects.value())
  {
    auto project = findProject(projectuuid);

    auto dataset = project->second->getDataset(query);
    addDatasetToResult(dataset, result);
  }
  return result;
}

void Core::addDatasetToResult(seerep_core_msgs::QueryResultProject& dataset, seerep_core_msgs::QueryResult& result)
{
  if (!dataset.dataOrInstanceUuids.empty())
  {
    result.queryResultProjects.push_back(dataset);
  }
}

} /* namespace seerep_core */
