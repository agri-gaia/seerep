#ifndef SEEREP_CORE_PROJECT_OVERVIEW_H_
#define SEEREP_CORE_PROJECT_OVERVIEW_H_

#include <functional>
#include <optional>
#include <filesystem>

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

// seerep-msgs
#include <seerep-msgs/boundingbox.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "project.h"

namespace seerep_core
{
class ProjectOverview
{
public:
  ProjectOverview(std::string datafolder);
  ~ProjectOverview();
  std::vector<std::optional<seerep::PointCloud2>> getPointCloud(const seerep::Boundingbox& bb);

  void addPointCloud(const seerep::PointCloud2& pointcloud2);
  std::string newProject();

private:
  void recreateProjects();

  std::string datafolder;

  std::string coordinatesystem;

  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::Project>, boost::hash<boost::uuids::uuid>> projects;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_PROJECT_OVERVIEW_H_
