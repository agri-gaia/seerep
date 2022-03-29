#ifndef SEEREP_CORE_CORE_H_
#define SEEREP_CORE_CORE_H_

#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <filesystem>
#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/dataset-indexable.h>
#include <seerep-msgs/project-info.h>
#include <seerep-msgs/query-result.h>
#include <seerep-msgs/query-tf.h>
#include <seerep-msgs/query.h>

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// seerep-core
#include "core-project.h"

namespace seerep_core
{
class Core
{
public:
  Core(std::string datafolder);
  ~Core();

  void addPointCloud(const seerep_core_msgs::DatasetIndexable& pointcloud);
  seerep_core_msgs::QueryResult getPointCloud(const seerep_core_msgs::Query& query);

  void addImage(const seerep_core_msgs::DatasetIndexable& image);
  seerep_core_msgs::QueryResult getImage(const seerep_core_msgs::Query& query);
  void addImageLabels(std::vector<std::string>& labels, const boost::uuids::uuid& msgUuid,
                      const boost::uuids::uuid& projectuuid);

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

  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::CoreProject>, boost::hash<boost::uuids::uuid>>
      m_projects;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_H_
