#include "seerep-core/pointcloud.h"

namespace seerep_core
{
Pointcloud::Pointcloud(const std::string& uuid, HighFive::Group& cloud_group) : m_uuid(uuid), m_cloud_group(cloud_group)
{
}

Pointcloud::~Pointcloud()
{
}

std::optional<seerep::PointCloud2> Pointcloud::getData(const seerep::Query& query)
{
  /*
  std::cout << "loading PC from pointclouds/" << m_id << std::endl;
  Eigen::Vector4f minPt, maxPt;
  getMinMaxFromBundingBox(minPt, maxPt, query.boundingbox());
  std::optional<seerep::PointCloud2> pc = m_hdf5_io->readPointCloud2(std::to_string(m_id) + "/rawdata");

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
  */

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

AabbHierarchy::AABB Pointcloud::getAABB()
{
  std::vector<float> bb;
  m_cloud_group.getAttribute(seerep_pb_io::PointCloudIO::BOUNDINGBOX).read(bb);

  return AabbHierarchy::AABB(AabbHierarchy::Point(bb[0], bb[1], bb[2]), AabbHierarchy::Point(bb[3], bb[4], bb[5]));
}

std::string Pointcloud::getUUID()
{
  return m_uuid;
}

} /* namespace seerep_core */
