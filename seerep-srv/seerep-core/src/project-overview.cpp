#include "seerep-core/project-overview.h"

namespace seerep_core
{
ProjectOverview::ProjectOverview(std::string datafolder) : datafolder(datafolder)
{
  coordinatesystem = "test";

  recreateProjects();
}
ProjectOverview::~ProjectOverview()
{
}

std::vector<std::vector<std::optional<seerep::PointCloud2>>>
ProjectOverview::getPointCloud(const seerep::Boundingbox& bb)
{
  std::vector<std::vector<std::optional<seerep::PointCloud2>>> result;
  for (auto& it : projects)
  {
    result.push_back(it.second->getPointCloud(bb));
  }

  return result;
}

void ProjectOverview::recreateProjects()
{
  for (const auto& entry : std::filesystem::directory_iterator(datafolder))
  {
    if (entry.path().filename().extension() == ".h5")
    {
      std::cout << "found " << entry.path().string() << " in HDF5 file." << std::endl;

      try
      {
        boost::uuids::string_generator gen;
        boost::uuids::uuid uuid = gen(entry.path().filename().stem().string());

        auto project = std::make_shared<Project>(uuid, entry.path().string());
        projects.insert(std::make_pair(uuid, project));
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
  std::string path = datafolder + "/" + filename + ".h5";

  auto project = std::make_shared<Project>(uuid, path, projectname);
  projects.insert(std::make_pair(uuid, project));

  return filename;
}

void ProjectOverview::addPointCloud(const seerep::PointCloud2& pointcloud2, boost::uuids::uuid uuid)
{
  projects.at(uuid)->addPointCloud(pointcloud2);
}

} /* namespace seerep_core */
