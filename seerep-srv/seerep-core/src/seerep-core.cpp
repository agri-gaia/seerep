#include "seerep-core/seerep-core.h"

namespace seerep_core
{
SeerepCore::SeerepCore(std::string datafolder) : m_datafolder(datafolder)
{
  recreateProjects();
}
SeerepCore::~SeerepCore()
{
}

seerep_core_msgs::QueryResult SeerepCore::getPointCloud(const seerep_core_msgs::Query& query)
{
  // std::vector<std::vector<std::optional<seerep::PointCloud2>>> result;

  // // search all projects
  // if (query.projectuuid().empty())
  // {
  //   for (auto& it : m_projects)
  //   {
  //     auto pc = it.second->getPointCloud(query);
  //     if (!pc.empty())
  //     {
  //       result.push_back(pc);
  //     }
  //   }
  // }
  // // Search only in project specified in query
  // else
  // {
  //   boost::uuids::uuid uuid;
  //   try
  //   {
  //     boost::uuids::string_generator gen;
  //     uuid = gen(query.projectuuid());
  //   }
  //   catch (const std::runtime_error& e)
  //   {
  //     std::cout << e.what() << std::endl;
  //   }
  //   auto project = m_projects.find(uuid);
  //   if (project != m_projects.end())
  //   {
  //     auto pc = project->second->getPointCloud(query);
  //     if (!pc.empty())
  //     {
  //       result.push_back(pc);
  //     }
  //   }
  // }

  // return result;
}

seerep_core_msgs::QueryResult SeerepCore::getImage(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;

  // search all projects
  if (query.header.uuidProject.is_nil())
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
    auto project = m_projects.find(query.header.uuidProject);
    if (project != m_projects.end())
    {
      auto img = project->second->getImage(query);
      if (!img.dataUuids.empty())
      {
        result.queryResultProjects.push_back(img);
      }
    }
  }

  return result;
}

void SeerepCore::recreateProjects()
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

        auto project = std::make_shared<Project>(uuid, entry.path().string());
        m_projects.insert(std::make_pair(uuid, project));
      }
      catch (const std::runtime_error& e)
      {
        std::cout << e.what() << std::endl;
      }
    }
  }
}

void SeerepCore::newProject(const seerep_core_msgs::ProjectInfo& projectInfo)
{
  std::string filename = boost::lexical_cast<std::string>(projectInfo.uuid);
  std::string path = m_datafolder + "/" + filename + ".h5";

  auto project = std::make_shared<Project>(projectInfo.uuid, path, projectInfo.name, projectInfo.frameId);
  m_projects.insert(std::make_pair(projectInfo.uuid, project));
}

std::vector<seerep_core_msgs::ProjectInfo> SeerepCore::getProjects()
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

boost::uuids::uuid SeerepCore::addPointCloud(const seerep_core_msgs::DatasetIndexable& pointcloud2)
{
  // m_projects.at(projectuuid)->addPointCloud(pointcloud2);
}

void SeerepCore::addImage(const seerep_core_msgs::DatasetIndexable& image)
{
  m_projects.at(image.header.uuidProject)->addImage(image);
}

void SeerepCore::addTF(const geometry_msgs::TransformStamped& tf, const boost::uuids::uuid& projectuuid)
{
  m_projects.at(projectuuid)->addTF(tf);
}

std::optional<geometry_msgs::TransformStamped> SeerepCore::getTF(const seerep_core_msgs::QueryTf& transformQuery)
{
  return m_projects.at(transformQuery.project)->getTF(transformQuery);
}

std::vector<std::string> SeerepCore::getFrames(const boost::uuids::uuid& projectuuid)
{
  return m_projects.at(projectuuid)->getFrames();
}

std::shared_ptr<std::mutex> SeerepCore::getHdf5FileMutex(const boost::uuids::uuid& projectuuid)
{
  return m_projects.at(projectuuid)->getHdf5FileMutex();
}
std::shared_ptr<HighFive::File> SeerepCore::getHdf5File(const boost::uuids::uuid& projectuuid)
{
  return m_projects.at(projectuuid)->getHdf5File();
}

} /* namespace seerep_core */
