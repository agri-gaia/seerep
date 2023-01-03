#include "seerep-hdf5-ros/hdf5-ros.h"

namespace seerep_hdf5_ros
{
Hdf5Ros::Hdf5Ros(std::shared_ptr<HighFive::File>& hdf5File, std::shared_ptr<std::mutex>& mutex,
                 const std::string& projectName, const std::string& projectFrameId)
  : Hdf5CoreGeneral(hdf5File, mutex)
{
  writeProjectname(projectName);
  writeProjectFrameId(projectFrameId);
}

void Hdf5Ros::saveHeader(const std::string& hdf5DataSetPath, const std_msgs::Header& header)
{
  if (exists(hdf5DataSetPath))
  {
    std::shared_ptr<HighFive::DataSet> imageDataSet = getHdf5DataSet(hdf5DataSetPath);

    writeAttributeToHdf5<uint32_t>(*imageDataSet, "header_seq", header.seq);
    writeAttributeToHdf5<uint64_t>(*imageDataSet, "header_stamp_sec", header.stamp.sec);
    writeAttributeToHdf5<uint64_t>(*imageDataSet, "header_stamp_nsec", header.stamp.nsec);
    writeAttributeToHdf5<std::string>(*imageDataSet, "header_frame_id", header.frame_id);
  }
}

void Hdf5Ros::saveImage(const sensor_msgs::Image& image)
{
  const std::string imageDataSetPath = "image/" + boost::lexical_cast<std::string>(boost::uuids::random_generator()());

  HighFive::DataSpace imageDataSpace = HighFive::DataSpace(image.height * image.width * 3);
  std::shared_ptr<HighFive::DataSet> imageDataSet = getHdf5DataSet<uint8_t>(imageDataSetPath, imageDataSpace);

  saveHeader(imageDataSetPath, image.header);

  writeAttributeToHdf5<uint32_t>(*imageDataSet, "height", image.height);
  writeAttributeToHdf5<uint32_t>(*imageDataSet, "width", image.width);
  writeAttributeToHdf5<std::string>(*imageDataSet, "encoding", image.encoding);
  writeAttributeToHdf5<bool>(*imageDataSet, "is_bigendian", image.is_bigendian);
  writeAttributeToHdf5<uint32_t>(*imageDataSet, "step", image.step);

  imageDataSet->write(image.data.data());
}

}  // namespace seerep_hdf5_ros
