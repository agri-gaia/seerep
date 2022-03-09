#ifndef SEEREP_CORE_SEEREP_CORE_H_
#define SEEREP_CORE_SEEREP_CORE_H_

#include <functional>
#include <optional>
#include <filesystem>

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

// seerep-msgs
#include <seerep-msgs/project-info.h>
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-tf.h>
#include <seerep-msgs/query-result.h>
#include <seerep-msgs/dataset-indexable.h>

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// seerep-core
#include "project.h"

namespace seerep_core
{
class SeerepCore
{
public:
  SeerepCore(std::string datafolder);
  ~SeerepCore();

  boost::uuids::uuid addPointCloud(const seerep_core_msgs::DatasetIndexable& pointcloud);
  seerep_core_msgs::QueryResult getPointCloud(const seerep_core_msgs::Query& query);

  void addImage(const seerep_core_msgs::DatasetIndexable& image);
  seerep_core_msgs::QueryResult getImage(const seerep_core_msgs::Query& query);

  void addTF(const geometry_msgs::TransformStamped& tf, const boost::uuids::uuid& projectuuid);
  std::optional<geometry_msgs::TransformStamped> getTF(const seerep_core_msgs::QueryTf& transformQuery);
  std::vector<std::string> getFrames(const boost::uuids::uuid& projectuuid);

  void newProject(const seerep_core_msgs::ProjectInfo& projectInfo);
  std::vector<seerep_core_msgs::ProjectInfo> getProjects();

  std::shared_ptr<std::mutex> getHdf5FileMutex(const boost::uuids::uuid& projectuuid);
  std::shared_ptr<HighFive::File> getHdf5File(const boost::uuids::uuid& projectuuid);

private:
  void recreateProjects();

  std::string m_datafolder;

  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::Project>, boost::hash<boost::uuids::uuid>>
      m_projects;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_SEEREP_CORE_H_
