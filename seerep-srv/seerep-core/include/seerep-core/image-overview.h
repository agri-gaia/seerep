#ifndef SEEREP_CORE_IMAGE_OVERVIEW_H_
#define SEEREP_CORE_IMAGE_OVERVIEW_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/boundingbox.pb.h>
#include <seerep-msgs/image.pb.h>
// #include <seerep-msgs/image_labeled.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "image.h"
#include "aabb-hierarchy.h"

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_core
{
class ImageOverview
{
public:
  ImageOverview();
  ImageOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io);
  ~ImageOverview();
  std::vector<std::optional<seerep::Image>> getData(const seerep::Query& bb);

  void addDataset(const seerep::Image& image);
  // void addDatasetLabeled(const seerep::ImageLabeled& imagelabeled);

private:
  void recreateDatasets();

  uint64_t data_count;

  std::string coordinatesystem;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;

  std::unordered_map<uint64_t, std::shared_ptr<seerep_core::Image>> m_datasets;

  AabbHierarchy::rtree m_rt;
  AabbHierarchy::timetree m_timetree;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_IMAGE_OVERVIEW_H_
