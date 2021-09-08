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

// std::vector<std::optional<seerep::PointCloud2>> ProjectOverview::getPointCloud(const seerep::Boundingbox& bb)
// {
// }

void ProjectOverview::recreateProjects()
{
  for (const auto& entry : std::filesystem::directory_iterator(datafolder))
  {
    if (entry.path().filename().extension() == ".h5")
    {
      std::cout << "found " << entry.path().string() << " in HDF5 file." << std::endl;

      boost::uuids::string_generator gen;
      boost::uuids::uuid uuid = gen(entry.path().filename().stem().string());

      auto project = std::make_shared<Project>(uuid, entry.path().string());
      projects.insert(std::make_pair(uuid, project));
    }
  }
}

std::string ProjectOverview::newProject()
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();

  std::string filename = boost::lexical_cast<std::string>(uuid);
  std::string path = datafolder + "/" + filename + ".h5";
  HighFive::File hdf5_file(path, HighFive::File::Create);

  auto project = std::make_shared<Project>(uuid, path);
  projects.insert(std::make_pair(uuid, project));

  return filename;
}

void ProjectOverview::addPointCloud(const seerep::PointCloud2& pointcloud2)
{
}

} /* namespace seerep_core */
