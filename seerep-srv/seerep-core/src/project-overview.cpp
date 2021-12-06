#include "seerep-core/project-overview.h"

namespace seerep_core
{
ProjectOverview::ProjectOverview(std::string datafolder) : m_datafolder(datafolder)
{
  m_coordinatesystem = "test";

  recreateProjects();
}
ProjectOverview::~ProjectOverview()
{
}

std::vector<std::vector<std::optional<seerep::PointCloud2>>> ProjectOverview::getPointCloud(const seerep::Query& query)
{
  std::vector<std::vector<std::optional<seerep::PointCloud2>>> result;
  for (auto& it : m_projects)
  {
    result.push_back(it.second->getPointCloud(query));
  }

  return result;
}

std::vector<std::vector<std::optional<seerep::Image>>> ProjectOverview::getImage(const seerep::Query& query)
{
  std::vector<std::vector<std::optional<seerep::Image>>> result;
  for (auto& it : m_projects)
  {
    result.push_back(it.second->getImage(query));
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

std::string ProjectOverview::newProject(std::string projectname)
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();

  std::string filename = boost::lexical_cast<std::string>(uuid);
  std::string path = m_datafolder + "/" + filename + ".h5";

  auto project = std::make_shared<Project>(uuid, path, projectname);
  m_projects.insert(std::make_pair(uuid, project));

  return filename;
}

void ProjectOverview::addPointCloud(const seerep::PointCloud2& pointcloud2, boost::uuids::uuid uuid)
{
  m_projects.at(uuid)->addPointCloud(pointcloud2);
}

void ProjectOverview::addPointCloudLabeled(const seerep::PointCloud2Labeled& pointcloud2labeled, boost::uuids::uuid uuid)
{
  m_projects.at(uuid)->addPointCloudLabeled(pointcloud2labeled);
}

void ProjectOverview::addImage(const seerep::Image& image, boost::uuids::uuid uuid)
{
  m_projects.at(uuid)->addImage(image);
}

} /* namespace seerep_core */
