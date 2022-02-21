#ifndef SEEREP_CORE_PROJECT_H_
#define SEEREP_CORE_PROJECT_H_

#include <functional>
#include <optional>

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
// seerep-hdf5
#include <seerep-hdf5/general-io.h>
#include <seerep-hdf5/ioTf.h>
#include <seerep-hdf5/pointcloud-io.h>
#include <seerep-hdf5/image-io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "pointcloud.h"
#include "pointcloud-overview.h"
#include "image.h"
#include "image-overview.h"
#include "tf.h"
#include "tf-overview.h"

namespace seerep_core
{
class Project
{
public:
  Project(boost::uuids::uuid& uuid, std::string path);
  Project(boost::uuids::uuid& uuid, std::string path, std::string projectname, std::string mapFrameId);
  ~Project();
  std::string getName();
  std::vector<std::optional<seerep::PointCloud2>> getPointCloud(const seerep::Query& query);
  std::vector<std::optional<seerep::Image>> getImage(const seerep::Query& query);

  void addPointCloud(const seerep::PointCloud2& pointcloud2);
  // void addPointCloudLabeled(const seerep::PointCloud2Labeled& pointcloud2Labeled);
  boost::uuids::uuid addImage(const seerep::Image& image);
  void addTF(const seerep::TransformStamped& tf);
  std::optional<seerep::TransformStamped> getTF(const seerep::TransformStampedQuery& transformQuery);
  std::vector<std::string> getFrames();

private:
  void createHdf5Io(boost::uuids::uuid& uuid, std::string path);
  void recreateDatatypes();

  boost::uuids::uuid m_id;

  std::string m_path;
  std::string m_projectname;
  std::string m_frameId;

  std::shared_ptr<std::mutex> m_write_mtx;
  std::shared_ptr<seerep_hdf5::GeneralIO> m_ioGeneral;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IOTf> m_ioTf;
  std::shared_ptr<seerep_hdf5::PointCloudIO> m_ioPointCloud;
  std::shared_ptr<seerep_hdf5::ImageIO> m_ioImage;

  std::shared_ptr<seerep_core::TFOverview> m_tfOverview;
  std::unique_ptr<seerep_core::PointcloudOverview> m_pointcloudOverview;
  std::unique_ptr<seerep_core::ImageOverview> m_imageOverview;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_PROJECT_H_
