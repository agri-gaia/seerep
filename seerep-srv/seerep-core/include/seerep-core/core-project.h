#ifndef SEEREP_CORE_CORE_PROJECT_H_
#define SEEREP_CORE_CORE_PROJECT_H_

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/dataset-indexable.h>
#include <seerep-msgs/query-result-project.h>
#include <seerep-msgs/query-tf.h>
#include <seerep-msgs/query.h>

// seerep-hdf5-core
#include <seerep-hdf5-core/hdf5-core-image.h>
#include <seerep-hdf5-core/hdf5-core-point-cloud.h>
#include <seerep-hdf5-core/hdf5-core-tf.h>

// seerep-core
#include "core-image.h"
#include "core-point-cloud.h"
#include "core-tf.h"

namespace seerep_core
{
class CoreProject
{
public:
  CoreProject(const boost::uuids::uuid& uuid, const std::string path);
  CoreProject(const boost::uuids::uuid& uuid, const std::string path, const std::string projectname,
              const std::string mapFrameId);
  ~CoreProject();

  std::string getName();
  std::string getFrameId();

  seerep_core_msgs::QueryResultProject getPointCloud(const seerep_core_msgs::Query& query);
  seerep_core_msgs::QueryResultProject getImage(const seerep_core_msgs::Query& query);

  void addPointCloud(const seerep_core_msgs::DatasetIndexable& dataset);
  void addImage(const seerep_core_msgs::DatasetIndexable& dataset);
  void addImageLabels(std::vector<std::string>& labels, const boost::uuids::uuid& msgUuid);

  // tf
  void addTF(const geometry_msgs::TransformStamped& tf);
  std::optional<geometry_msgs::TransformStamped> getTF(const seerep_core_msgs::QueryTf& transformQuery);
  std::vector<std::string> getFrames();

  std::shared_ptr<std::mutex> getHdf5FileMutex();
  std::shared_ptr<HighFive::File> getHdf5File();

private:
  void createHdf5Io(boost::uuids::uuid& uuid, std::string path);
  void recreateDatatypes();

  boost::uuids::uuid m_id;

  std::string m_path;
  std::string m_projectname;
  std::string m_frameId;

  std::shared_ptr<std::mutex> m_write_mtx;
  std::shared_ptr<HighFive::File> m_hdf5_file;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> m_ioGeneral;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> m_ioTf;
  std::shared_ptr<seerep_hdf5_core::Hdf5CorePointCloud> m_ioPointCloud;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreImage> m_ioImage;

  std::shared_ptr<seerep_core::CoreTf> m_tfOverview;
  std::unique_ptr<seerep_core::CorePointCloud> m_pointcloudOverview;
  std::unique_ptr<seerep_core::CoreImage> m_imageOverview;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_PROJECT_H_
