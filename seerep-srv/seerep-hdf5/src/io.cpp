#include "seerep-hdf5/io.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5
{
SeerepHDF5IO::SeerepHDF5IO(HighFive::File& file) : file(file)
{
}

void SeerepHDF5IO::writeHeaderAttributes(HighFive::DataSet& data_set, const seerep::Header& header)
{
  if (!data_set.hasAttribute(HEADER_STAMP_SECONDS))
    data_set.createAttribute(HEADER_STAMP_SECONDS, header.stamp().seconds());
  else
    data_set.getAttribute(HEADER_STAMP_SECONDS).write(header.stamp().seconds());

  if (!data_set.hasAttribute(HEADER_STAMP_NANOS))
    data_set.createAttribute(HEADER_STAMP_NANOS, header.stamp().nanos());
  else
    data_set.getAttribute(HEADER_STAMP_NANOS).write(header.stamp().nanos());

  if (!data_set.hasAttribute(HEADER_FRAME_ID))
    data_set.createAttribute(HEADER_FRAME_ID, header.frame_id());
  else
    data_set.getAttribute(HEADER_FRAME_ID).write(header.frame_id());

  if (!data_set.hasAttribute(HEADER_SEQ))
    data_set.createAttribute(HEADER_SEQ, header.seq());
  else
    data_set.getAttribute(HEADER_SEQ).write(header.seq());
}

seerep::Header SeerepHDF5IO::readHeaderAttributes(HighFive::DataSet& data_set)
{
  seerep::Header header;

  int64_t seconds;
  int32_t nanos;
  uint32_t seq;

  data_set.getAttribute(HEADER_FRAME_ID).read(header.mutable_frame_id());

  data_set.getAttribute(HEADER_STAMP_SECONDS).read(seconds);
  data_set.getAttribute(HEADER_STAMP_NANOS).read(nanos);
  data_set.getAttribute(HEADER_SEQ).read(seq);

  header.set_seq(seq);
  header.mutable_stamp()->set_seconds(seconds);
  header.mutable_stamp()->set_nanos(nanos);

  return header;
}

void SeerepHDF5IO::writeImage(const std::string& id, const seerep::Image& image)
{
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space({ image.height(), image.step() });

  if (!file.exist(id))
  {
    std::cout << "data id " << id << " does not exist! Creat new dataset in hdf5" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.createDataSet<uint8_t>(id, data_space));
    data_set_ptr->createAttribute(HEIGHT, image.height());
    data_set_ptr->createAttribute(WIDTH, image.width());
    data_set_ptr->createAttribute(ENCODING, image.encoding());
    data_set_ptr->createAttribute(IS_BIGENDIAN, image.is_bigendian());
    data_set_ptr->createAttribute(ROW_STEP, image.step());
  }
  else
  {
    std::cout << "data id " << id << " already exists!" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.getDataSet(id));
    data_set_ptr->getAttribute(HEIGHT).write(image.height());
    data_set_ptr->getAttribute(WIDTH).write(image.width());
    data_set_ptr->getAttribute(ENCODING).write(image.encoding());
    data_set_ptr->getAttribute(IS_BIGENDIAN).write(image.is_bigendian());
    data_set_ptr->getAttribute(ROW_STEP).write(image.step());
  }

  const uint8_t* begin = reinterpret_cast<const uint8_t*>(image.data().c_str());
  std::vector<std::vector<uint8_t>> tmp;
  tmp.resize(image.height());

  for (int i = 0; i < image.height(); i++)
  {
    const uint8_t* row = begin + i * image.step();
    tmp[i].reserve(image.step());
    std::copy_n(row, image.step(), std::back_inserter(tmp[i]));
  }
  data_set_ptr->write(tmp);
  writeHeaderAttributes(*data_set_ptr, image.header());
  file.flush();
}

std::optional<seerep::Image> SeerepHDF5IO::readImage(const std::string& id)
{
  if (!file.exist(id))
    return std::nullopt;

  HighFive::DataSet data_set = file.getDataSet(id);

  seerep::Image image;
  data_set.read(image.mutable_data());
  *image.mutable_header() = readHeaderAttributes(data_set);
  return image;
}

void SeerepHDF5IO::writePointFieldAttributes(
    HighFive::DataSet& data_set, const google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField)
{
  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;
  for (int i = 0; i < repeatedPointField.size(); i++)
  {
    names.push_back(repeatedPointField.at(i).name());
    offsets.push_back(repeatedPointField.at(i).offset());
    datatypes.push_back(static_cast<uint8_t>(repeatedPointField.at(i).datatype()));
    counts.push_back(repeatedPointField.at(i).count());
  }

  if (!data_set.hasAttribute(FIELD_NAME))
    data_set.createAttribute(FIELD_NAME, names);
  else
    data_set.getAttribute(FIELD_NAME).write(names);

  if (!data_set.hasAttribute(FIELD_OFFSET))
    data_set.createAttribute(FIELD_OFFSET, offsets);
  else
    data_set.getAttribute(FIELD_OFFSET).write(offsets);

  if (!data_set.hasAttribute(FIELD_DATATYPE))
    data_set.createAttribute(FIELD_DATATYPE, datatypes);
  else
    data_set.getAttribute(FIELD_DATATYPE).write(datatypes);

  if (!data_set.hasAttribute(FIELD_COUNT))
    data_set.createAttribute(FIELD_COUNT, counts);
  else
    data_set.getAttribute(FIELD_COUNT).write(counts);
}

google::protobuf::RepeatedPtrField<seerep::PointField>
SeerepHDF5IO::readPointFieldAttributes(HighFive::DataSet& data_set)
{
  google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField;

  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;

  data_set.getAttribute(FIELD_NAME).read(names);
  data_set.getAttribute(FIELD_OFFSET).read(offsets);
  data_set.getAttribute(FIELD_DATATYPE).read(datatypes);
  data_set.getAttribute(FIELD_COUNT).read(counts);

  for (int i = 0; i < names.size(); i++)
  {
    seerep::PointField point_field;

    point_field.set_name(names.at(i));
    point_field.set_offset(offsets.at(i));
    point_field.set_datatype(static_cast<seerep::PointField::Datatype>(datatypes.at(i)));
    point_field.set_count(counts.at(i));

    *repeatedPointField.Add() = point_field;
  }

  return repeatedPointField;
}

void SeerepHDF5IO::writePointCloud2(const std::string& id, const seerep::PointCloud2& pointcloud2)
{
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space({ pointcloud2.height(), pointcloud2.row_step() });

  if (!file.exist(id))
  {
    std::cout << "data id " << id << " does not exist! Creat new dataset in hdf5" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.createDataSet<uint8_t>(id, data_space));
    data_set_ptr->createAttribute(HEIGHT, pointcloud2.height());
    data_set_ptr->createAttribute(WIDTH, pointcloud2.width());
    data_set_ptr->createAttribute(IS_BIGENDIAN, pointcloud2.is_bigendian());
    data_set_ptr->createAttribute(POINT_STEP, pointcloud2.point_step());
    data_set_ptr->createAttribute(ROW_STEP, pointcloud2.row_step());
    data_set_ptr->createAttribute(IS_DENSE, pointcloud2.is_dense());
  }
  else
  {
    std::cout << "data id " << id << " already exists!" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.getDataSet(id));
    data_set_ptr->getAttribute(HEIGHT).write(pointcloud2.height());
    data_set_ptr->getAttribute(WIDTH).write(pointcloud2.width());
    data_set_ptr->getAttribute(IS_BIGENDIAN).write(pointcloud2.is_bigendian());
    data_set_ptr->getAttribute(POINT_STEP).write(pointcloud2.point_step());
    data_set_ptr->getAttribute(ROW_STEP).write(pointcloud2.row_step());
    data_set_ptr->getAttribute(IS_DENSE).write(pointcloud2.is_dense());
  }

  writePointFieldAttributes(*data_set_ptr, pointcloud2.fields());

  const uint8_t* begin = reinterpret_cast<const uint8_t*>(pointcloud2.data().c_str());
  std::vector<std::vector<uint8_t>> tmp;
  tmp.resize(pointcloud2.height());

  for (int i = 0; i < pointcloud2.height(); i++)
  {
    const uint8_t* row = begin + i * pointcloud2.row_step();
    tmp[i].reserve(pointcloud2.row_step());
    std::copy_n(row, pointcloud2.row_step(), std::back_inserter(tmp[i]));
  }
  data_set_ptr->write(tmp);
  writeHeaderAttributes(*data_set_ptr, pointcloud2.header());
  file.flush();
}

std::optional<seerep::PointCloud2> SeerepHDF5IO::readPointCloud2(const std::string& id)
{
  if (!file.exist(id))
    return std::nullopt;

  HighFive::DataSet data_set = file.getDataSet(id);

  seerep::PointCloud2 pointcloud2;
  data_set.read(pointcloud2.mutable_data());
  *pointcloud2.mutable_header() = readHeaderAttributes(data_set);

  uint32_t height, width, point_step, row_step;
  bool is_bigendian, is_dense;
  data_set.getAttribute(HEIGHT).read(height);
  data_set.getAttribute(WIDTH).read(width);
  data_set.getAttribute(IS_BIGENDIAN).read(is_bigendian);
  data_set.getAttribute(POINT_STEP).read(point_step);
  data_set.getAttribute(ROW_STEP).read(row_step);
  data_set.getAttribute(IS_DENSE).read(is_dense);

  pointcloud2.set_height(height);
  pointcloud2.set_width(width);
  pointcloud2.set_is_bigendian(is_bigendian);
  pointcloud2.set_point_step(point_step);
  pointcloud2.set_row_step(row_step);
  pointcloud2.set_is_dense(is_dense);

  *pointcloud2.mutable_fields() = readPointFieldAttributes(data_set);

  return pointcloud2;
}

void SeerepHDF5IO::writePointAttributes(HighFive::DataSet& data_set, const seerep::Point& point,
                                        const std::string& prefix)
{
  if (!data_set.hasAttribute(prefix + X))
    data_set.createAttribute(prefix + X, point.x());
  else
    data_set.getAttribute(prefix + X).write(point.x());

  if (!data_set.hasAttribute(prefix + Y))
    data_set.createAttribute(prefix + Y, point.y());
  else
    data_set.getAttribute(prefix + Y).write(point.y());

  if (!data_set.hasAttribute(prefix + Z))
    data_set.createAttribute(prefix + Z, point.z());
  else
    data_set.getAttribute(prefix + Z).write(point.z());
}

void SeerepHDF5IO::writePoint(const std::string& id, const seerep::Point& point)
{
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space(0);
  if (!file.exist(id))
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.createDataSet<uint8_t>(id, data_space));
  else
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.getDataSet(id));
  writePointAttributes(*data_set_ptr, point);
  file.flush();
}

void SeerepHDF5IO::writeQuaternionAttributes(HighFive::DataSet& data_set, const seerep::Quaternion& quaternion,
                                             const std::string& prefix)
{
  if (!data_set.hasAttribute(prefix + X))
    data_set.createAttribute(prefix + X, quaternion.x());
  else
    data_set.getAttribute(prefix + X).write(quaternion.x());

  if (!data_set.hasAttribute(prefix + Y))
    data_set.createAttribute(prefix + Y, quaternion.y());
  else
    data_set.getAttribute(prefix + Y).write(quaternion.y());

  if (!data_set.hasAttribute(prefix + Z))
    data_set.createAttribute(prefix + Z, quaternion.z());
  else
    data_set.getAttribute(prefix + Z).write(quaternion.z());

  if (!data_set.hasAttribute(prefix + W))
    data_set.createAttribute(prefix + W, quaternion.w());
  else
    data_set.getAttribute(prefix + W).write(quaternion.w());
}

void SeerepHDF5IO::writeQuaternion(const std::string& id, const seerep::Quaternion& quaternion)
{
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space(0);
  if (!file.exist(id))
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.createDataSet<uint8_t>(id, data_space));
  else
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.getDataSet(id));
  writeQuaternionAttributes(*data_set_ptr, quaternion);
  file.flush();
}

void SeerepHDF5IO::writePose(const std::string& id, const seerep::Pose& pose)
{
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space(0);
  if (!file.exist(id))
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.createDataSet<uint8_t>(id, data_space));
  else
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.getDataSet(id));
  writePointAttributes(*data_set_ptr, pose.position(), POSITION + "/");
  writeQuaternionAttributes(*data_set_ptr, pose.orientation(), ORIENTATION + "/");
  file.flush();
}

void SeerepHDF5IO::writePoseStamped(const std::string& id, const seerep::PoseStamped& pose)
{
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space(0);
  if (!file.exist(id))
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.createDataSet<uint8_t>(id, data_space));
  else
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.getDataSet(id));
  writeHeaderAttributes(*data_set_ptr, pose.header());
  writePointAttributes(*data_set_ptr, pose.pose().position(), POSE + "/" + POSITION + "/");
  writeQuaternionAttributes(*data_set_ptr, pose.pose().orientation(), POSE + "/" + ORIENTATION + "/");
  file.flush();
}

} /* namespace seerep_hdf5 */
