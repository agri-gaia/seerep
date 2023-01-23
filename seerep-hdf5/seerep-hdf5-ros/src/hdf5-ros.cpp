#include "seerep-hdf5-ros/hdf5-ros.h"

namespace seerep_hdf5_ros
{
Hdf5Ros::Hdf5Ros(std::shared_ptr<HighFive::File>& hdf5File, std::shared_ptr<std::mutex>& mutex,
                 const std::string& projectName, const std::string& projectFrameId)
  : seerep_hdf5_core::Hdf5CoreGeneral(hdf5File, mutex)
  , seerep_hdf5_core::Hdf5CoreTf(hdf5File, mutex)
  , hdf5File_(hdf5File)
{
  writeProjectname(projectName);
  writeProjectFrameId(projectFrameId);
}

// Assumption is that the dataset to write the header to already exists, otherwise the function will do nothing.
void Hdf5Ros::saveMessage(const std::string& hdf5GroupPath, const std_msgs::Header& header)
{
  if (exists(hdf5GroupPath))
  {
    std::shared_ptr<HighFive::Group> imageDataGroup = getHdf5Group(hdf5GroupPath);

    writeAttributeToHdf5<uint32_t>(*imageDataGroup, "header_seq", header.seq);
    writeAttributeToHdf5<uint64_t>(*imageDataGroup, "header_stamp_sec", header.stamp.sec);
    writeAttributeToHdf5<uint64_t>(*imageDataGroup, "header_stamp_nsec", header.stamp.nsec);
    writeAttributeToHdf5<std::string>(*imageDataGroup, "header_frame_id", header.frame_id);
  }
}

void Hdf5Ros::saveMessage(const sensor_msgs::Image& image)
{
  const std::string imageDataGroupPath =
      "image/" + boost::lexical_cast<std::string>(boost::uuids::random_generator()());
  const std::string imageDataSetPath = imageDataGroupPath + "/rawdata";

  HighFive::DataSpace imageDataSpace = HighFive::DataSpace(image.height * image.width * 3);
  std::shared_ptr<HighFive::Group> imageDataGroup = getHdf5Group(imageDataGroupPath);
  std::shared_ptr<HighFive::DataSet> imageDataSet = getHdf5DataSet<uint8_t>(imageDataSetPath, imageDataSpace);

  saveMessage(imageDataGroupPath, image.header);

  writeAttributeToHdf5<uint32_t>(*imageDataGroup, "height", image.height);
  writeAttributeToHdf5<uint32_t>(*imageDataGroup, "width", image.width);
  writeAttributeToHdf5<std::string>(*imageDataSet, "encoding", image.encoding);
  writeAttributeToHdf5<bool>(*imageDataSet, "is_bigendian", image.is_bigendian);
  writeAttributeToHdf5<uint32_t>(*imageDataGroup, "step", image.step);

  imageDataSet->write(image.data.data());
}

void Hdf5Ros::saveMessage(const sensor_msgs::PointCloud2& pointcloud2)
{
  const std::string pointcloud2DataSetPath =
      "pointcloud2/" + boost::lexical_cast<std::string>(boost::uuids::random_generator()());

  HighFive::DataSpace pointcloud2DataSpace = HighFive::DataSpace(pointcloud2.data.size());
  std::shared_ptr<HighFive::DataSet> pointcloud2DataSet =
      getHdf5DataSet<uint8_t>(pointcloud2DataSetPath, pointcloud2DataSpace);

  saveMessage(pointcloud2DataSetPath, pointcloud2.header);
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

void Hdf5Ros::saveMessage(const tf2_msgs::TFMessage& transformation)
{
  for (auto transform : transformation.transforms)
  {
    const std::string datagroupPath =
        seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF + "/" + transform.header.frame_id + "_" + transform.child_frame_id;
    std::shared_ptr<HighFive::Group> group;
    if (!hdf5File_->exist(datagroupPath))
    {
      group = std::make_shared<HighFive::Group>(hdf5File_->createGroup(datagroupPath));
      group->createAttribute<uint64_t>(seerep_hdf5_core::Hdf5CoreTf::SIZE, 0);
      writeAttributeToHdf5<std::string>(*group, "PARENT_FRAME", transform.header.frame_id);
      writeAttributeToHdf5<std::string>(*group, "CHILD_FRAME", transform.child_frame_id);
    }
    else
    {
      group = std::make_shared<HighFive::Group>(hdf5File_->getGroup(datagroupPath));
    }

    writeTimestamp(datagroupPath, { transform.header.stamp.sec, transform.header.stamp.nsec });
    writeTranslation(datagroupPath, { transform.transform.translation.x, transform.transform.translation.y,
                                      transform.transform.translation.z });
    writeRotation(datagroupPath, { transform.transform.rotation.x, transform.transform.rotation.y,
                                   transform.transform.rotation.z, transform.transform.rotation.w });
    uint64_t size = readAttributeFromHdf5<uint64_t>(std::to_string(transform.header.seq), *group,
                                                    seerep_hdf5_core::Hdf5CoreTf::SIZE);
    writeAttributeToHdf5<uint64_t>(*group, seerep_hdf5_core::Hdf5CoreTf::SIZE, size + 1);
    hdf5File_->flush();
  }
}
}  // namespace seerep_hdf5_ros
