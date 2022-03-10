// #include "seerep-core/core-pointcloud.h"

// // uuids
// #include <boost/uuid/uuid.hpp>
// #include <boost/uuid/random_generator.hpp>
// #include <boost/uuid/string_generator.hpp>
// #include <boost/uuid/uuid_io.hpp>

// namespace seerep_core
// {
// CorePointCloud::CorePointCloud()
// {
// }
// CorePointCloud::CorePointCloud(std::shared_ptr<seerep_pb_io::PointCloudIO> hdf5_io)
//   : m_hdf5_io(hdf5_io), data_count(0)
// {
//   coordinatesystem = "test";

//   recreateDatasets();
// }
// CorePointCloud::~CorePointCloud()
// {
// }

// void CorePointCloud::recreateDatasets()
// {
//   // std::vector<std::string> pcs = m_hdf5_io->getGroupDatasets("pointclouds");
//   std::map<std::string, HighFive::Group> clouds = m_hdf5_io->getPointClouds();

//   // TODO clear trees
//   data_count = 0;

//   for (auto cloud : clouds)
//   {
//     std::cout << "found " << cloud.first << " in HDF5 file." << std::endl;

//     std::shared_ptr<Pointcloud> cloud_io_ptr = std::make_shared<Pointcloud>(cloud.first, cloud.second);

//     m_datasets.insert(std::make_pair(data_count, cloud_io_ptr));
//     m_rt.insert(std::make_pair(cloud_io_ptr->getAABB(), data_count));
//     data_count++;
//   }
// }

// std::vector<std::optional<seerep::PointCloud2>> CorePointCloud::getData(const seerep::Query& query)
// {
//   std::vector<std::optional<seerep::PointCloud2>> result;

//   const seerep::Point min = query.boundingbox().point_min();
//   const seerep::Point max = query.boundingbox().point_max();

//   AabbHierarchy::AABB aabb(AabbHierarchy::Point(min.x(), min.y(), min.z()),
//                            AabbHierarchy::Point(max.x(), max.y(), max.z()));

//   std::vector<AabbHierarchy::AabbIdPair> rt_result;

//   // do the semantic query first and extend this query parameter by check if id is in semantic result
//   m_rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result));

//   for (auto& r : rt_result)
//   {
//     std::optional<seerep::PointCloud2> pc = m_datasets.at(r.second)->getData(query);

//     if (pc)
//     {
//       std::cout << "checked " << pc.value().data() << std::endl;
//       result.push_back(pc);
//     }
//   }

//   return result;
// }

// void CorePointCloud::addDataset(const seerep::PointCloud2& cloud)
// {
//   boost::uuids::uuid uuid;
//   if (cloud.header().uuid_msgs().empty())
//   {
//     uuid = boost::uuids::random_generator()();
//   }
//   else
//   {
//     boost::uuids::string_generator gen;
//     uuid = gen(cloud.header().uuid_msgs());
//   }

//   std::shared_ptr<HighFive::Group> cloud_group_ptr =
//       m_hdf5_io->writePointCloud2(boost::lexical_cast<std::string>(uuid), cloud);

//   std::shared_ptr<Pointcloud> cloud_io_ptr =
//       std::make_shared<Pointcloud>(boost::lexical_cast<std::string>(uuid), *cloud_group_ptr);

//   m_datasets.insert(std::make_pair(data_count, cloud_io_ptr));
//   m_rt.insert(std::make_pair(cloud_io_ptr->getAABB(), data_count));

//   data_count++;
// }

// } /* namespace seerep_core */
