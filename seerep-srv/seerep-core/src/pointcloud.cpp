#include "seerep-core/pointcloud.h"

namespace seerep_core
{
Pointcloud::Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
                       const seerep::PointCloud2& pointcloud2, const uint64_t& id)
  : coordinatesystemParent(coordinatesystemParent), hdf5_io(hdf5_io), id(id)
{
  hdf5_io->writePointCloud2("pointclouds/" + std::to_string(id) + "/rawdata", pointcloud2);
}
Pointcloud::Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
                       const uint64_t& id)
  : coordinatesystemParent(coordinatesystemParent), hdf5_io(hdf5_io), id(id)
{
}
Pointcloud::Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
                       const seerep::PointCloud2Labeled& pointcloud2labeled, const uint64_t& id)
  : coordinatesystemParent(coordinatesystemParent), hdf5_io(hdf5_io), id(id)
{
  hdf5_io->writePointCloud2("pointclouds/" + std::to_string(id) + "/rawdata", pointcloud2labeled.pointcloud());
  // TODO
  std::cout << "labels to hdf5 not implemented!" << std::endl;
}
Pointcloud::~Pointcloud()
{
}
std::optional<seerep::PointCloud2> Pointcloud::getData(const seerep::Boundingbox bb)
{
  std::cout << "loading PC from pointclouds/" << id << std::endl;
  Eigen::Vector4f minPt, maxPt;
  getBoundingBox(minPt, maxPt, bb);
  std::optional<seerep::PointCloud2> pc = hdf5_io->readPointCloud2("pointclouds/" + std::to_string(id) + "/rawdata");

  if (pc)
  {
    std::cout << "converting PC to ROS" << std::endl;
    sensor_msgs::PointCloud2 pc_ros = seerep_ros_conversions::toROS(pc.value());
    std::cout << "ROS size: " << pc_ros.height * pc_ros.row_step << std::endl;

    std::cout << "converting PC to PCL" << std::endl;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pc_ros, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    std::cout << "PCL size: " << temp_cloud->size() << std::endl;

    std::cout << "filtering PC" << std::endl;
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(minPt);
    boxFilter.setMax(maxPt);
    boxFilter.setInputCloud(temp_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    boxFilter.filter(*filtered_cloud);
    std::cout << "PCL filtered size: " << filtered_cloud->size() << std::endl;

    std::cout << "converting PC to Proto" << std::endl;
    pcl::toPCLPointCloud2(*filtered_cloud, pcl_pc2);
    sensor_msgs::PointCloud2 pc2_msg;
    pcl_conversions::fromPCL(pcl_pc2, pc2_msg);
    std::cout << "ROS filtered size: " << pc2_msg.height * pc2_msg.row_step << std::endl;
    return seerep_ros_conversions::toProto(pc2_msg);
  }

  return std::nullopt;
}

void Pointcloud::getBoundingBox(Eigen::Vector4f& minPt, Eigen::Vector4f& maxPt, const seerep::Boundingbox& bb)
{
  if (bb.point_max().x() < bb.point_min().x())
  {
    maxPt(0) = bb.point_min().x();
    minPt(0) = bb.point_max().x();
  }
  else
  {
    maxPt(0) = bb.point_max().x();
    minPt(0) = bb.point_min().x();
  }
  if (bb.point_max().y() < bb.point_min().y())
  {
    maxPt(1) = bb.point_min().y();
    minPt(1) = bb.point_max().y();
  }
  else
  {
    maxPt(1) = bb.point_max().y();
    minPt(1) = bb.point_min().y();
  }
  if (bb.point_max().z() < bb.point_min().z())
  {
    maxPt(2) = bb.point_min().z();
    minPt(2) = bb.point_max().z();
  }
  else
  {
    maxPt(2) = bb.point_max().z();
    minPt(2) = bb.point_min().z();
  }

  maxPt(3) = 1.0f;
  minPt(3) = 1.0f;
}
} /* namespace seerep_core */
