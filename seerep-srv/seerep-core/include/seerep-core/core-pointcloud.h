// #ifndef SEEREP_CORE_CORE_POINTCLOUD_H_
// #define SEEREP_CORE_CORE_POINTCLOUD_H_

// #include <functional>
// #include <optional>

// // seerep-msgs
// #include <seerep-msgs/aabb.h>
// #include <seerep-msgs/query.h>
// #include <seerep-msgs/query-result.h>

// // seerep-pb-io
// #include <seerep-pb-io/pointcloud-io-core.h>
// // seerep-conversion
// #include <seerep_ros_conversions/conversions.h>

// // seerep-core
// #include "tf-overview.h"

// // uuid
// #include <boost/uuid/uuid.hpp>             // uuid class
// #include <boost/uuid/uuid_generators.hpp>  // generators
// #include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
// #include <boost/lexical_cast.hpp>
// #include <boost/functional/hash.hpp>

// namespace seerep_core
// {
// class CorePointCloud
// {
// public:
//   CorePointCloud();
//   CorePointCloud(std::shared_ptr<seerep_pb_io::PointCloudIO> hdf5_io);
//   ~CorePointCloud();
//   std::vector<std::optional<seerep::PointCloud2>> getData(const seerep::Query& query);

//   void addDataset(const seerep::PointCloud2& pointcloud2);
//   // void addDatasetLabeled(const seerep::PointCloud2Labeled& pointcloud2labeled);

// private:
//   void recreateDatasets();

//   uint64_t data_count;

//   std::string coordinatesystem;
//   std::shared_ptr<seerep_pb_io::PointCloudIO> m_hdf5_io;

//   std::unordered_map<uint64_t, std::shared_ptr<seerep_core::Pointcloud>> m_datasets;

//   AabbHierarchy::rtree m_rt;
// };

// } /* namespace seerep_core */

// #endif  // SEEREP_CORE_CORE_POINTCLOUD_H_
