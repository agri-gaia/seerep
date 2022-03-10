#ifndef SEEREP_CORE_CORE_PROJECT_H_
#define SEEREP_CORE_CORE_PROJECT_H_

#include <functional>
#include <optional>

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// seerep-msgs
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-tf.h>
#include <seerep-msgs/query-result-project.h>
#include <seerep-msgs/dataset-indexable.h>

// seerep-core-io
#include <seerep-io-core/io-core-image.h>
#include <seerep-io-core/io-core-tf.h>

// seerep-core
// #include "pointcloud.h"
// #include "pointcloud-overview.h"
// #include "image.h"
#include "core-image.h"
// #include "tf.h"
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
  // seerep_core_msgs::QueryResultProject getPointCloud(const seerep_core_msgs::Query& query);
  seerep_core_msgs::QueryResultProject getImage(const seerep_core_msgs::Query& query);

  // boost::uuids::uuid addPointCloud(const seerep_core_msgs::DatasetIndexable& pointcloud);
  void addImage(const seerep_core_msgs::DatasetIndexable& pointcloud);

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
  std::shared_ptr<seerep_io_core::IoCoreGeneral> m_ioGeneral;
  std::shared_ptr<seerep_io_core::IoCoreTf> m_ioTf;
  // std::shared_ptr<seerep_core_io::PointCloudIOCore> m_ioPointCloud;
  std::shared_ptr<seerep_io_core::IoCoreImage> m_ioImage;

  std::shared_ptr<seerep_core::CoreTf> m_tfOverview;
  // std::unique_ptr<seerep_core::PointcloudOverview> m_pointcloudOverview;
  std::unique_ptr<seerep_core::CoreImage> m_imageOverview;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_PROJECT_H_
