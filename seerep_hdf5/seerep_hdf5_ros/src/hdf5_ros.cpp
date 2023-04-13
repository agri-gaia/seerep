#include "seerep_hdf5_ros/hdf5_ros.h"

namespace seerep_hdf5_ros
{
Hdf5Ros::Hdf5Ros(const std::string& path, const std::string& filename, const std::string& projectFrameId,
                 const std::string& projectName)
  : path_{ path }, filename_{ filename }
{
  auto write_mtx = std::make_shared<std::mutex>();
  hdf5File_ =
      std::make_shared<HighFive::File>(path_ + filename_ + ".h5", HighFive::File::ReadWrite | HighFive::File::Create);
  auto ioGeneral = seerep_hdf5_core::Hdf5CoreGeneral(hdf5File_, write_mtx);
  ioGeneral.writeProjectFrameId(projectFrameId);
  ioGeneral.writeProjectname(projectName);
}

bool Hdf5Ros::dumpImage(const sensor_msgs::Image& image) const
{
  const std::string datasetPath = "images/" + boost::lexical_cast<std::string>(boost::uuids::random_generator()());

  H5Easy::dump(*hdf5File_, datasetPath, std::move(image.data));

  this->dumpHeader(image.header, datasetPath);

  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "encoding", image.encoding);
  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "height", image.height);
  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "width", image.width);
  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "is_bigendian", image.is_bigendian);
  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "step", image.step);

  return true;
}

bool Hdf5Ros::dumpHeader(const std_msgs::Header& header, const std::string& datasetPath) const
{
  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "seq", header.seq);
  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "stamp_seconds", header.stamp.sec);
  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "stamp_nanoseconds", header.stamp.nsec);
  H5Easy::dumpAttribute(*hdf5File_, datasetPath, "frame_id", header.frame_id);

  return true;
}

}  // namespace seerep_hdf5_ros
