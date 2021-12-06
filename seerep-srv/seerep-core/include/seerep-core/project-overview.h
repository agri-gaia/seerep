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
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
#include <seerep-msgs/point_cloud_2_labeled.pb.h>
#include <seerep-msgs/image.pb.h>
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
  std::vector<std::vector<std::optional<seerep::PointCloud2>>> getPointCloud(const seerep::Query& query);

  void addPointCloud(const seerep::PointCloud2& pointcloud2, boost::uuids::uuid uuid);
  void addPointCloudLabeled(const seerep::PointCloud2Labeled& pointcloud2labeled, boost::uuids::uuid uuid);
  void addImage(const seerep::Image& image, boost::uuids::uuid uuid);

  std::vector<std::vector<std::optional<seerep::Image>>> getImage(const seerep::Query& query);

  std::string newProject(std::string projectname);

private:
  void recreateProjects();

  std::string m_datafolder;

  std::string m_coordinatesystem;

  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::Project>, boost::hash<boost::uuids::uuid>>
      m_projects;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_PROJECT_OVERVIEW_H_
