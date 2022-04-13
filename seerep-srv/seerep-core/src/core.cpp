#include "seerep-core/core.h"

namespace seerep_core
{
Core::Core(std::string datafolder) : m_datafolder(datafolder)
{
  recreateProjects();
}
Core::~Core()
{
}

seerep_core_msgs::QueryResult Core::getPointCloud(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;

  // search all projects
  if (query.header.uuidProject.is_nil())
  {
    for (auto& it : m_projects)
    {
      auto pc = it.second->getPointCloud(query);
      if (!pc.dataUuids.empty())
      {
        result.queryResultProjects.push_back(pc);
      }
    }
  }
  // Search only in project specified in query
  else
  {
    auto project = m_projects.find(query.header.uuidProject);
    if (project != m_projects.end())
    {
      auto pc = project->second->getPointCloud(query);
      if (!pc.dataUuids.empty())
      {
        result.queryResultProjects.push_back(pc);
      }
    }
    // if not found throw error
    else
    {
      throw std::runtime_error("project " + boost::lexical_cast<std::string>(query.header.uuidProject) +
                               "does not exist!");
    };
  }

  return result;
}

seerep_core_msgs::QueryResult Core::getImage(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;

  // search all projects
  if (query.projects.empty())
  {
    for (auto& it : m_projects)
    {
      auto img = it.second->getImage(query);
      if (!img.dataUuids.empty())
      {
        result.queryResultProjects.push_back(img);
      }
    }
  }
  // Search only in project specified in query
  else
  {
    for (auto projectuuid : query.projects)
    {
      auto project = m_projects.find(projectuuid);
      if (project != m_projects.end())
      {
        auto img = project->second->getImage(query);
        if (!img.dataUuids.empty())
        {
          result.queryResultProjects.push_back(img);
        }
      }
      // if not found throw error
      else
      {
        throw std::runtime_error("project " + boost::lexical_cast<std::string>(query.header.uuidProject) +
                                 "does not exist!");
      };
    }
  }

  return result;
}

void Core::recreateProjects()
{
  for (const auto& entry : std::filesystem::directory_iterator(m_datafolder))
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

void Core::newProject(const seerep_core_msgs::ProjectInfo& projectInfo)
{
  std::string filename = boost::lexical_cast<std::string>(projectInfo.uuid);
  std::string path = m_datafolder + "/" + filename + ".h5";

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

void Core::addPointCloud(const seerep_core_msgs::DatasetIndexable& dataset)
{
  // find the project based on its uuid
  auto project = m_projects.find(dataset.header.uuidProject);
  // if project was found add pointcloud2
  if (project != m_projects.end())
  {
    return project->second->addPointCloud(dataset);
  }
  // if not found throw error
  else
  {
    throw std::runtime_error("project " + boost::lexical_cast<std::string>(dataset.header.uuidProject) +
                             "does not exist!");
  };
}

void Core::addImage(const seerep_core_msgs::DatasetIndexable& dataset)
{
  // find the project based on its uuid
  auto project = m_projects.find(dataset.header.uuidProject);
  // if project was found add image
  if (project != m_projects.end())
  {
    return project->second->addImage(dataset);

    if (!dataset.labelsWithInstances.empty())
    {
      // add to core-instances
    }
  }
  // if not found throw error
  else
  {
    throw std::runtime_error("project " + boost::lexical_cast<std::string>(dataset.header.uuidProject) +
                             "does not exist!");
  };
}

void Core::addImageLabels(std::vector<std::string>& labels, const boost::uuids::uuid& msgUuid,
                          const boost::uuids::uuid& projectuuid)
{
  // find the project based on its uuid
  auto project = m_projects.find(projectuuid);
  // if project was found add image
  if (project != m_projects.end())
  {
    return project->second->addImageLabels(labels, msgUuid);
  }
  // if not found throw error
  else
  {
    throw std::runtime_error("project " + boost::lexical_cast<std::string>(projectuuid) + "does not exist!");
  };
}

void Core::addTF(const geometry_msgs::TransformStamped& tf, const boost::uuids::uuid& projectuuid)
{
  // find the project based on its uuid
  auto project = m_projects.find(projectuuid);
  // if project was found add tf
  if (project != m_projects.end())
  {
    return project->second->addTF(tf);
  }
  // if not found throw error
  else
  {
    throw std::runtime_error("project " + boost::lexical_cast<std::string>(projectuuid) + "does not exist!");
  };
}

std::optional<geometry_msgs::TransformStamped> Core::getTF(const seerep_core_msgs::QueryTf& transformQuery)
{
  // find the project based on its uuid
  auto project = m_projects.find(transformQuery.project);
  // if project was found call function and return result
  if (project != m_projects.end())
  {
    return project->second->getTF(transformQuery);
  }
  // if not found return empty optional
  else
  {
    return std::nullopt;
  };
}

std::vector<std::string> Core::getFrames(const boost::uuids::uuid& projectuuid)
{
  // find the project based on its uuid
  auto project = m_projects.find(projectuuid);
  // if project was found call function and return result
  if (project != m_projects.end())
  {
    return project->second->getFrames();
  }
  // if not found return empty vector
  else
  {
    return {};
  };
}

std::shared_ptr<std::mutex> Core::getHdf5FileMutex(const boost::uuids::uuid& projectuuid)
{
  // find the project based on its uuid
  auto project = m_projects.find(projectuuid);
  // if project was found return pointer to mutex
  if (project != m_projects.end())
  {
    return project->second->getHdf5FileMutex();
  }
  // if not found return null pointer
  else
  {
    return nullptr;
  }
}
std::shared_ptr<HighFive::File> Core::getHdf5File(const boost::uuids::uuid& projectuuid)
{
  // find the project based on its uuid
  auto project = m_projects.find(projectuuid);
  // if project was found return pointer to HighFive::File
  if (project != m_projects.end())
  {
    return project->second->getHdf5File();
  }
  // if not found return null pointer
  else
  {
    return nullptr;
  }
}

} /* namespace seerep_core */
