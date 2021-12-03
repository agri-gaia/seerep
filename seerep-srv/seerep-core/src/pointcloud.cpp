#include "seerep-core/pointcloud.h"

namespace seerep_core
{
Pointcloud::Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
                       const seerep::PointCloud2& pointcloud2, const uint64_t& id)
  : m_coordinatesystemParent(coordinatesystemParent), m_hdf5_io(hdf5_io), m_id(id)
{
  m_hdf5_io->writePointCloud2("pointclouds/" + std::to_string(m_id) + "/rawdata", pointcloud2);

  m_aabb = calcAABB(pointcloud2);
  m_hdf5_io->writeAABB("pointclouds/" + std::to_string(m_id), m_aabb);
}
Pointcloud::Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
                       const uint64_t& id)
  : m_coordinatesystemParent(coordinatesystemParent), m_hdf5_io(hdf5_io), m_id(id)
{
  m_hdf5_io->readAABB("pointclouds/" + std::to_string(m_id), m_aabb);
}
Pointcloud::Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
                       const seerep::PointCloud2Labeled& pointcloud2labeled, const uint64_t& id)
  : m_coordinatesystemParent(coordinatesystemParent), m_hdf5_io(hdf5_io), m_id(id)
{
  m_hdf5_io->writePointCloud2Labeled("pointclouds/" + std::to_string(m_id), pointcloud2labeled);

  m_aabb = calcAABB(pointcloud2labeled.pointcloud());
  m_hdf5_io->writeAABB("pointclouds/" + std::to_string(m_id), m_aabb);
}
Pointcloud::~Pointcloud()
{
}
std::optional<seerep::PointCloud2> Pointcloud::getData(const seerep::Query& query)
{
  std::cout << "loading PC from pointclouds/" << m_id << std::endl;
  Eigen::Vector4f minPt, maxPt;
  getMinMaxFromBundingBox(minPt, maxPt, query.boundingbox());
  std::optional<seerep::PointCloud2> pc =
      m_hdf5_io->readPointCloud2("pointclouds/" + std::to_string(m_id) + "/rawdata");

  if (pc)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    protoToPcl(pc.value(), temp_cloud);

    std::cout << "filtering PC" << std::endl;
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(minPt);
    boxFilter.setMax(maxPt);
    boxFilter.setInputCloud(temp_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    boxFilter.filter(*filtered_cloud);
    std::cout << "PCL filtered size: " << filtered_cloud->size() << std::endl;

    seerep::PointCloud2 result_pc;
    pclToProto(filtered_cloud, result_pc);
    return result_pc;
  }

  return std::nullopt;
}

void Pointcloud::getMinMaxFromBundingBox(Eigen::Vector4f& minPt, Eigen::Vector4f& maxPt, const seerep::Boundingbox& bb)
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

void Pointcloud::protoToPcl(const seerep::PointCloud2& pc_proto, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl)
{
  std::cout << "converting PC to ROS" << std::endl;
  sensor_msgs::PointCloud2 pc_ros = seerep_ros_conversions::toROS(pc_proto);
  std::cout << "ROS size: " << pc_ros.height * pc_ros.row_step << std::endl;

  std::cout << "converting PC to PCL" << std::endl;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(pc_ros, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pc_pcl);
  std::cout << "PCL size: " << pc_pcl->size() << std::endl;
}

void Pointcloud::pclToProto(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_pcl, seerep::PointCloud2& pc_proto)
{
  std::cout << "converting PC to Proto" << std::endl;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*pc_pcl, pcl_pc2);
  sensor_msgs::PointCloud2 pc2_msg;
  pcl_conversions::fromPCL(pcl_pc2, pc2_msg);
  std::cout << "ROS filtered size: " << pc2_msg.height * pc2_msg.row_step << std::endl;
  pc_proto = seerep_ros_conversions::toProto(pc2_msg);
}

AabbHierarchy::AABB Pointcloud::getAABB()
{
  return m_aabb;
}

uint64_t Pointcloud::getID()
{
  return m_id;
}

AabbHierarchy::AABB Pointcloud::calcAABB(const seerep::PointCloud2& pointcloud2)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  protoToPcl(pointcloud2, temp_cloud);
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(temp_cloud);
  feature_extractor.compute();

  pcl::PointXYZ min_point_AABB, max_point_AABB;
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  return AabbHierarchy::AABB(AabbHierarchy::Point(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z),
                             AabbHierarchy::Point(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z));
}

} /* namespace seerep_core */
