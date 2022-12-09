#include "seerep_core/core.h"

namespace seerep_core
{
Core::Core(std::string dataFolder, bool loadHdf5Files) : m_dataFolder(dataFolder)
{
  if (loadHdf5Files)
  {
    loadProjectsInFolder();
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

      auto instances = project->second->getInstances(query);
      if (!instances.dataOrInstanceUuids.empty())
      {
        result.queryResultProjects.push_back(instances);
      }
    }
  }

  return result;
}

void Core::loadProjectsInFolder()
{
  for (const auto& entry : std::filesystem::directory_iterator(m_dataFolder))
  {
    if (entry.path().filename().extension() == ".h5")
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "found " << entry.path().string();

      try
      {
        boost::uuids::string_generator gen;
        boost::uuids::uuid uuid = gen(entry.path().filename().stem().string());

        if (m_projects.find(uuid) == m_projects.end())
        {
          auto project = std::make_shared<CoreProject>(uuid, entry.path().string());
          m_projects.insert(std::make_pair(uuid, project));
        }
      }
      catch (const std::runtime_error& e)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
      }
    }
  }
}

void Core::deleteProject(boost::uuids::uuid uuid)
{
  auto project = m_projects.find(uuid);

  if (project != m_projects.end())
  {
    m_projects.erase(project);

    std::string filename = boost::lexical_cast<std::string>(uuid);
    std::string path = m_dataFolder + "/" + filename + ".h5";

    std::filesystem::remove(path);
  }
};

void Core::createProject(const seerep_core_msgs::ProjectInfo& projectInfo)
{
  std::string filename = boost::lexical_cast<std::string>(projectInfo.uuid);
  std::string path = m_dataFolder + "/" + filename + ".h5";

  auto project = std::make_shared<CoreProject>(projectInfo.uuid, path, projectInfo.name, projectInfo.frameId,
                                               projectInfo.geodetCoords);
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
    projectinfo.geodetCoords = it->second->getGeodeticCoordinates();

    projectInfos.push_back(projectinfo);
  }

  return projectInfos;
}

void Core::addDataset(const seerep_core_msgs::DatasetIndexable& dataset)
{
  auto project = findProject(dataset.header.uuidProject);

  project->second->addDataset(dataset);
}

void Core::addLabels(const seerep_core_msgs::Datatype& datatype,
                     const std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>&
                         labelWithInstancePerCategory,
                     const boost::uuids::uuid& msgUuid, const boost::uuids::uuid& projectuuid)
{
  auto project = findProject(projectuuid);

  project->second->addLabels(datatype, labelWithInstancePerCategory, msgUuid);
}

void Core::addTF(const geometry_msgs::TransformStamped& tf, const boost::uuids::uuid& projectuuid)
{
  auto project = findProject(projectuuid);
  project->second->addTF(tf);
}

void Core::addCameraIntrinsics(const seerep_core_msgs::camera_intrinsics& ci, const boost::uuids::uuid& projectuuid)
{
  auto project = findProject(projectuuid);
  project->second->addCameraIntrinsics(ci);
}

std::optional<seerep_core_msgs::camera_intrinsics>
Core::getCameraIntrinsics(const seerep_core_msgs::camera_intrinsics_query& ci_query)
{
  try
  {
    auto project = findProject(ci_query.uuidProject);
    return project->second->getCameraIntrinsics(ci_query.uuidCameraIntrinsics);
  }
  catch (const std::runtime_error& e)
  {
    return std::nullopt;
  }
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
  return checkSize(result, query.maxNumData);
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
  return checkSize(result, query.maxNumData);
}

seerep_core_msgs::QueryResult Core::checkSize(const seerep_core_msgs::QueryResult& queryResult, uint maxNum)
{
  seerep_core_msgs::QueryResult queryResultFiltered(queryResult);

  uint64_t overallResultSize = 0;
  for (auto& queryResultProject : queryResultFiltered.queryResultProjects)
  {
    overallResultSize += queryResultProject.dataOrInstanceUuids.size();
  }

  if (maxNum > 0 && overallResultSize > maxNum)
  {
    float factor = maxNum / (float)overallResultSize;
    for (auto& queryResultProject : queryResultFiltered.queryResultProjects)
    {
      queryResultProject.dataOrInstanceUuids =
          std::vector<boost::uuids::uuid>(queryResultProject.dataOrInstanceUuids.begin(),
                                          queryResultProject.dataOrInstanceUuids.begin() +
                                              (int)std::round(queryResultProject.dataOrInstanceUuids.size() * factor));
    }
  }
  return queryResultFiltered;
}

void Core::addDatasetToResult(seerep_core_msgs::QueryResultProject& dataset, seerep_core_msgs::QueryResult& result)
{
  if (!dataset.dataOrInstanceUuids.empty())
  {
    result.queryResultProjects.push_back(dataset);
  }
}

seerep_core_msgs::AabbTime Core::getOverallTimeInterval(boost::uuids::uuid uuid)
{
  auto project = findProject(uuid);
  return project->second->getTimeBounds();
}

seerep_core_msgs::AABB Core::getOverallBound(boost::uuids::uuid uuid)
{
  auto project = findProject(uuid);
  return project->second->getSpatialBounds();
}

} /* namespace seerep_core */
