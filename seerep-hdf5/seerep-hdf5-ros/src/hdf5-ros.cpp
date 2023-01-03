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

// Assumption is that the dataset to write the header to already exists, otherwise the function will do nothing.
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

void Hdf5Ros::savePointCloud2(const sensor_msgs::PointCloud2& pointcloud2)
{
  const std::string pointcloud2DataSetPath =
      "pointcloud2/" + boost::lexical_cast<std::string>(boost::uuids::random_generator()());

  HighFive::DataSpace pointcloud2DataSpace = HighFive::DataSpace(pointcloud2.data.size());
  std::shared_ptr<HighFive::DataSet> pointcloud2DataSet =
      getHdf5DataSet<uint8_t>(pointcloud2DataSetPath, pointcloud2DataSpace);

  saveHeader(pointcloud2DataSetPath, pointcloud2.header);
  writeAttributeToHdf5<uint32_t>(*pointcloud2DataSet, "height", pointcloud2.height);
  writeAttributeToHdf5<uint32_t>(*pointcloud2DataSet, "width", pointcloud2.width);

  std::vector<std::string> fieldsNames;
  std::vector<uint32_t> fieldsOffsets;
  std::vector<uint8_t> fieldsDatatypes;
  std::vector<uint32_t> fieldsCount;

  for (auto field : pointcloud2.fields)
  {
    fieldsNames.push_back(field.name);
    fieldsOffsets.push_back(field.offset);
    fieldsDatatypes.push_back(field.datatype);
    fieldsCount.push_back(field.count);
  }

  writeAttributeToHdf5<std::vector<std::string>>(*pointcloud2DataSet, "fields_names", fieldsNames);
  writeAttributeToHdf5<std::vector<uint32_t>>(*pointcloud2DataSet, "fields_offsets", fieldsOffsets);
  writeAttributeToHdf5<std::vector<uint8_t>>(*pointcloud2DataSet, "fields_datatypes", fieldsDatatypes);
  writeAttributeToHdf5<std::vector<uint32_t>>(*pointcloud2DataSet, "fields_count", fieldsCount);

  writeAttributeToHdf5<bool>(*pointcloud2DataSet, "is_bigendian", pointcloud2.is_bigendian);
  writeAttributeToHdf5<uint32_t>(*pointcloud2DataSet, "point_step", pointcloud2.point_step);
  writeAttributeToHdf5<uint32_t>(*pointcloud2DataSet, "row_step", pointcloud2.row_step);

  pointcloud2DataSet->write(pointcloud2.data.data());

  writeAttributeToHdf5<bool>(*pointcloud2DataSet, "is_dense", pointcloud2.is_dense);
}

void Hdf5Ros::saveTransformation(const geometry_msgs::Transform& transform)
{
}

void Hdf5Ros::saveTransformationStamped(const geometry_msgs::TransformStamped& transformation)
{
}
}  // namespace seerep_hdf5_ros
