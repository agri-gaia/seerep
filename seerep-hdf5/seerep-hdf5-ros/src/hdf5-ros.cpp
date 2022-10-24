#include "seerep-hdf5-ros/hdf5-ros.h"

namespace seerep_hdf5_ros
{
Hdf5Ros::Hdf5Ros(const std::string& basePath) : basePath_{ basePath }
{
  hdf5_file_ =
      std::make_shared<HighFive::File>(basePath_ + "test.h5", HighFive::File::ReadWrite | HighFive::File::Create);
}

bool Hdf5Ros::dumpImage(const sensor_msgs::Image& image)
{
  const std::string imageName = "images/" + std::to_string(image.header.stamp.sec);

  H5Easy::dump(*hdf5_file_, imageName, std::move(image.data));
  H5Easy::dumpAttribute(*hdf5_file_, imageName, "encoding", image.encoding);

  return true;
}

}  // namespace seerep_hdf5_ros
