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
      std::cout << "found " << entry.path().string() << " in HDF5 file." << std::endl;

      try
      {
        boost::uuids::string_generator gen;
        boost::uuids::uuid uuid = gen(entry.path().filename().stem().string());

        auto project = std::make_shared<CoreProject>(uuid, entry.path().string());
        m_projects.insert(std::make_pair(uuid, project));
      }
      catch (const std::runtime_error& e)
      {
        std::cout << e.what() << std::endl;
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

void Core::addPointCloud(const seerep_core_msgs::DatasetIndexable& pointcloud2)
{
  // find the project based on its uuid
  auto project = m_projects.find(pointcloud2.header.uuidProject);
  // if project was found add pointcloud2
  if (project != m_projects.end())
  {
    return project->second->addPointCloud(pointcloud2);
  }
  // if not found throw error
  else
  {
    throw std::runtime_error("project " + boost::lexical_cast<std::string>(pointcloud2.header.uuidProject) +
                             "does not exist!");
  };
}

void Core::addImage(const seerep_core_msgs::DatasetIndexable& image)
{
  // find the project based on its uuid
  auto project = m_projects.find(image.header.uuidProject);
  // if project was found add image
  if (project != m_projects.end())
  {
    return project->second->addImage(image);
  }
  // if not found throw error
  else
  {
    throw std::runtime_error("project " + boost::lexical_cast<std::string>(image.header.uuidProject) +
                             "does not exist!");
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
