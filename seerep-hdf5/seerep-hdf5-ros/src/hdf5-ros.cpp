#include "seerep-hdf5-ros/hdf5-ros.h"

namespace seerep_hdf5_ros
{
Hdf5Ros::Hdf5Ros(const std::string& path, const std::string& filename) : path_{ path }, filename_{ filename }
{
  hdf5_file_ =
      std::make_shared<HighFive::File>(path_ + filename_ + ".h5", HighFive::File::ReadWrite | HighFive::File::Create);
}

bool Hdf5Ros::dumpImage(const sensor_msgs::Image& image) const
{
  const std::string dataset_path = "images/" + boost::lexical_cast<std::string>(boost::uuids::random_generator()());

  H5Easy::dump(*hdf5_file_, dataset_path, std::move(image.data));

  this->dumpHeader(image.header, dataset_path);

  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "encoding", image.encoding);
  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "height", image.height);
  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "width", image.width);
  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "is_bigendian", image.is_bigendian);
  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "step", image.step);

  return true;
}

bool Hdf5Ros::dumpHeader(const std_msgs::Header& header, const std::string& dataset_path) const
{
  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "seq", header.seq);
  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "stamp_seconds", header.stamp.sec);
  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "stamp_nanoseconds", header.stamp.nsec);
  H5Easy::dumpAttribute(*hdf5_file_, dataset_path, "frame_id", header.frame_id);

  return true;
}

}  // namespace seerep_hdf5_ros
