#ifndef SEEREP_CORE_PROJECT_H_
#define SEEREP_CORE_PROJECT_H_

#include <functional>
#include <optional>

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// seerep-msgs
#include <seerep-msgs/boundingbox.pb.h>
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
#include <seerep-msgs/point_cloud_2_labeled.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "pointcloud.h"
#include "pointcloud-overview.h"
#include "image.h"
#include "image-overview.h"

namespace seerep_core
{
class Project
{
public:
  Project(boost::uuids::uuid& uuid, std::string path);
  Project(boost::uuids::uuid& uuid, std::string path, std::string projectname);
  ~Project();
  std::vector<std::optional<seerep::PointCloud2>> getPointCloud(const seerep::Boundingbox& bb);
  std::vector<std::optional<seerep::Image>> getImage(const seerep::Query& query);

  void addPointCloud(const seerep::PointCloud2& pointcloud2);
  void addPointCloudLabeled(const seerep::PointCloud2Labeled& pointcloud2Labeled);

private:
  void createHdf5Io(boost::uuids::uuid& uuid, std::string path);
  void recreateDatatypes();

  boost::uuids::uuid m_id;

  std::string m_path;
  std::string m_projectname;
  std::string m_coordinatesystem;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;

  seerep_core::PointcloudOverview m_pointcloudOverview;
  seerep_core::ImageOverview m_imageOverview;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_PROJECT_H_
