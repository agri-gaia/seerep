#include "seerep-core/project-overview.h"

namespace seerep_core
{
ProjectOverview::ProjectOverview(std::string datafolder) : m_datafolder(datafolder)
{
  recreateProjects();
}
ProjectOverview::~ProjectOverview()
{
}

std::vector<std::vector<std::optional<seerep::PointCloud2>>> ProjectOverview::getPointCloud(const seerep::Query& query)
{
  std::vector<std::vector<std::optional<seerep::PointCloud2>>> result;

  // search all projects
  if (query.projectuuid().empty())
  {
    for (auto& it : m_projects)
    {
      auto pc = it.second->getPointCloud(query);
      if (!pc.empty())
      {
        result.push_back(pc);
      }
    }
  }
  // Search only in project specified in query
  else
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(query.projectuuid());
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
    auto project = m_projects.find(uuid);
    if (project != m_projects.end())
    {
      auto pc = project->second->getPointCloud(query);
      if (!pc.empty())
      {
        result.push_back(pc);
      }
    }
  }

  return result;
}

std::vector<std::vector<std::optional<seerep::Image>>> ProjectOverview::getImage(const seerep::Query& query)
{
  std::vector<std::vector<std::optional<seerep::Image>>> result;

  // search all projects
  if (query.projectuuid().empty())
  {
    for (auto& it : m_projects)
    {
      auto img = it.second->getImage(query);
      if (!img.empty())
      {
        result.push_back(img);
      }
    }
  }
  // Search only in project specified in query
  else
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(query.projectuuid());
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
    auto project = m_projects.find(uuid);
    if (project != m_projects.end())
    {
      auto img = project->second->getImage(query);
      if (!img.empty())
      {
        result.push_back(img);
      }
    }
  }

  return result;
}

void ProjectOverview::recreateProjects()
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

void ProjectOverview::newProject(std::string projectname, std::string mapFrameId, seerep::ProjectInfo* projectInfo)
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();

  std::string filename = boost::lexical_cast<std::string>(uuid);
  std::string path = m_datafolder + "/" + filename + ".h5";

  auto project = std::make_shared<Project>(uuid, path, projectname, mapFrameId);
  m_projects.insert(std::make_pair(uuid, project));

  projectInfo->set_uuid(filename);
  projectInfo->set_name(projectname);
}

void ProjectOverview::getProjects(seerep::ProjectInfos* projectInfos)
{
  for (auto it = m_projects.begin(); it != m_projects.end(); ++it)
  {
    seerep::ProjectInfo* projectinfo = projectInfos->add_projects();
    projectinfo->set_name(it->second->getName());
    projectinfo->set_uuid(boost::lexical_cast<std::string>(it->first));
  }
}

void ProjectOverview::addPointCloud(const seerep::PointCloud2& pointcloud2, boost::uuids::uuid projectuuid)
{
  m_projects.at(projectuuid)->addPointCloud(pointcloud2);
}

boost::uuids::uuid ProjectOverview::addImage(const seerep::Image& image, boost::uuids::uuid projectuuid)
{
  return m_projects.at(projectuuid)->addImage(image);
}

void ProjectOverview::addTF(const seerep::TransformStamped& tf, boost::uuids::uuid projectuuid)
{
  m_projects.at(projectuuid)->addTF(tf);
}

std::optional<seerep::TransformStamped> ProjectOverview::getTF(const seerep::TransformStampedQuery& transformQuery,
                                                               boost::uuids::uuid projectuuid)
{
  return m_projects.at(projectuuid)->getTF(transformQuery);
}

std::vector<std::string> ProjectOverview::getFrames(boost::uuids::uuid projectuuid)
{
  return m_projects.at(projectuuid)->getFrames();
}

} /* namespace seerep_core */
