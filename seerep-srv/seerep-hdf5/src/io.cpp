#include "seerep-hdf5/io.h"
#include "seerep-hdf5/point_cloud2_iterator.hpp"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5
{
SeerepHDF5IO::SeerepHDF5IO(HighFive::File& file) : m_file(file)
{
}

template <typename T>
void SeerepHDF5IO::writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField,
                                  T value)
{
  if (!dataSetPtr->hasAttribute(attributeField))
    dataSetPtr->createAttribute(attributeField, value);
  else
    dataSetPtr->getAttribute(attributeField).write(value);
}

template <typename T>
T SeerepHDF5IO::getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr,
                             std::string attributeField)
{
  T attributeValue;
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->getAttribute(attributeField).read(attributeValue);
  }
  else
  {
    throw std::invalid_argument("id " + id + " has no attribute " + attributeField);
  }
  return attributeValue;
}

void SeerepHDF5IO::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField)
{
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->deleteAttribute(attributeField);
  }
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
  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + RAWDATA;

  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space({ image.height(), image.width(), image.step() / image.width() });

  if (!m_file.exist(hdf5DatasetRawDataPath))
  {
    std::cout << "data id " << hdf5DatasetRawDataPath << " does not exist! Creat new dataset in hdf5" << std::endl;
    data_set_ptr =
        std::make_shared<HighFive::DataSet>(m_file.createDataSet<uint8_t>(hdf5DatasetRawDataPath, data_space));
  }
  else
  {
    std::cout << "data id " << hdf5DatasetRawDataPath << " already exists!" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetRawDataPath));
  }

  writeAttribute<uint32_t>(data_set_ptr, HEIGHT, image.height());
  writeAttribute<uint32_t>(data_set_ptr, WIDTH, image.width());
  writeAttribute<std::string>(data_set_ptr, ENCODING, image.encoding());
  writeAttribute<bool>(data_set_ptr, IS_BIGENDIAN, image.is_bigendian());
  writeAttribute<uint32_t>(data_set_ptr, POINT_STEP, image.step());
  writeAttribute<uint32_t>(data_set_ptr, ROW_STEP, image.row_step());

  if (image.encoding() == "rgb8" || image.encoding() == "8UC3")
  {
    writeAttribute<std::string>(data_set_ptr, CLASS, "IMAGE");
    writeAttribute<std::string>(data_set_ptr, "IMAGE_VERSION", "1.2");
    writeAttribute<std::string>(data_set_ptr, "IMAGE_SUBCLASS", "IMAGE_TRUECOLOR");
    writeAttribute<std::string>(data_set_ptr, "INTERLACE_MODE", "INTERLACE_PIXEL");
  }
  else
  {
    deleteAttribute(data_set_ptr, CLASS);
    deleteAttribute(data_set_ptr, "IMAGE_VERSION");
    deleteAttribute(data_set_ptr, "IMAGE_SUBCLASS");
    deleteAttribute(data_set_ptr, "INTERLACE_MODE");
  }

  const uint8_t* begin = reinterpret_cast<const uint8_t*>(image.data().c_str());

  std::vector<std::vector<std::vector<uint8_t>>> tmp;
  tmp.resize(image.height());

  int pixel_step = image.step() / image.width();

  for (int row = 0; row < image.height(); row++)
  {
    tmp.at(row).resize(image.width());
    for (int col = 0; col < image.width(); col++)
    {
      const uint8_t* pxl = begin + row * image.step() + col * pixel_step;
      tmp.at(row).at(col).reserve(pixel_step);
      std::copy_n(pxl, pixel_step, std::back_inserter(tmp.at(row).at(col)));
    }
  }

  data_set_ptr->write(tmp);
  writeHeaderAttributes(*data_set_ptr, image.header());

  writeBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id, image.labels_bb());
  writeLabelsGeneral(HDF5_GROUP_IMAGE, id, image.labels_general());

  m_file.flush();
}

// void SeerepHDF5IO::writeImageLabeled(const std::string& id, const seerep::ImageLabeled& imageLabeled)
// {
//   writeImage(id, imageLabeled.image());
//   writeBoundingBox2DLabeled(id, imageLabeled.labels());
// }

std::optional<seerep::Image> SeerepHDF5IO::readImage(const std::string& id)
{
  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + RAWDATA;

  if (!m_file.exist(hdf5DatasetRawDataPath))
    return std::nullopt;

  std::cout << "loading " << hdf5DatasetRawDataPath << std::endl;

  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetRawDataPath));

  seerep::Image image;

  try
  {
    image.set_height(getAttribute<uint32_t>(id, data_set_ptr, HEIGHT));
    image.set_width(getAttribute<uint32_t>(id, data_set_ptr, WIDTH));
    image.set_encoding(getAttribute<std::string>(id, data_set_ptr, ENCODING));
    image.set_is_bigendian(getAttribute<bool>(id, data_set_ptr, IS_BIGENDIAN));
    image.set_step(getAttribute<uint32_t>(id, data_set_ptr, POINT_STEP));
    image.set_row_step(getAttribute<uint32_t>(id, data_set_ptr, ROW_STEP));
  }
  catch (const std::invalid_argument e)
  {
    std::cout << "error: " << e.what() << std::endl;
    return std::nullopt;
  }
  std::vector<std::vector<std::vector<uint8_t>>> read_data;
  data_set_ptr->read(read_data);

  int pixel_step = image.step() / image.width();

  // TODO: write into protobuf data buffer
  uint8_t data[image.height()][image.width()][pixel_step];

  for (int row = 0; row < image.height(); row++)
  {
    for (int col = 0; col < image.width(); col++)
    {
      std::copy(read_data.at(row).at(col).begin(), read_data.at(row).at(col).end(), data[row][col]);
    }
  }

  // std::cout << "read_data:" << std::endl;
  // int j = 0;
  // for (const auto& i : read_data)
  // {
  //   std::cout << unsigned(i) << ' ';
  //   j++;
  //   // if (j > 50)
  //   // break;
  // }
  image.set_data(data, sizeof(data));

  *image.mutable_header() = readHeaderAttributes(*data_set_ptr);
  auto labelsBB = readBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id);
  if (labelsBB)
  {
    *image.mutable_labels_bb() = labelsBB.value();
  }
  auto labelsGeneral = readLabelsGeneral(HDF5_GROUP_IMAGE, id);
  if (labelsGeneral)
  {
    *image.mutable_labels_general() = labelsGeneral.value();
  }
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

// void SeerepHDF5IO::writePointCloud2Labeled(const std::string& id, const seerep::PointCloud2Labeled& pointcloud2Labeled)
// {
//   writePointCloud2(id + "/" + RAWDATA, pointcloud2Labeled.pointcloud());

//   writeBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, id, pointcloud2Labeled.labels());
// }

void SeerepHDF5IO::writeAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file.exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file.getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file.getName());
  }
  std::cout << "get group " << id << std::endl;
  HighFive::Group group = m_file.getGroup(id);

  std::vector<float> aabbPoints{ aabb.min_corner().get<0>(), aabb.min_corner().get<1>(), aabb.min_corner().get<2>(),
                                 aabb.max_corner().get<0>(), aabb.max_corner().get<1>(), aabb.max_corner().get<2>() };

  std::cout << "write AABB as attribute" << std::endl;
  if (!group.hasAttribute(AABB_FIELD))
    group.createAttribute(AABB_FIELD, aabbPoints);
  else
    group.getAttribute(AABB_FIELD).write(aabbPoints);

  m_file.flush();
}

void SeerepHDF5IO::readAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file.exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file.getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file.getName());
  }
  std::cout << "get group " << id << std::endl;
  HighFive::Group group = m_file.getGroup(id);
  if (group.hasAttribute(AABB_FIELD))
  {
    std::vector<float> aabbPoints;
    group.getAttribute(AABB_FIELD).read(aabbPoints);

    aabb.min_corner().set<0>(aabbPoints.at(0));
    aabb.min_corner().set<1>(aabbPoints.at(1));
    aabb.min_corner().set<2>(aabbPoints.at(2));
    aabb.max_corner().set<0>(aabbPoints.at(3));
    aabb.max_corner().set<1>(aabbPoints.at(4));
    aabb.max_corner().set<2>(aabbPoints.at(5));
  }
}

bool SeerepHDF5IO::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file.exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file.getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file.getName());
  }
  std::cout << "get group " << id << std::endl;
  HighFive::Group group = m_file.getGroup(id);
  return group.hasAttribute(AABB_FIELD);
}

int64_t SeerepHDF5IO::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return readTime(datatypeGroup, uuid + "/" + RAWDATA);
}

int64_t SeerepHDF5IO::readTime(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file.exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file.getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file.getName());
  }

  int64_t time;
  switch (m_file.getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::Group group = m_file.getGroup(id);
      if (group.hasAttribute(HEADER_STAMP_SECONDS))
      {
        group.getAttribute(HEADER_STAMP_SECONDS).read(time);
        return time;
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_SECONDS);
      }
    };

    case HighFive::ObjectType::Dataset:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::DataSet dataset = m_file.getDataSet(id);
      if (dataset.hasAttribute(HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(HEADER_STAMP_SECONDS).read(time);
        return time;
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_SECONDS);
      }
    };
    default:
      return std::numeric_limits<uint64_t>::min();
  }
}

void SeerepHDF5IO::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& time)
{
  writeTime(datatypeGroup, uuid + "/" + RAWDATA, time);
}

void SeerepHDF5IO::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& time)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file.exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file.getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file.getName());
  }

  switch (m_file.getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::Group group = m_file.getGroup(id);
      if (group.hasAttribute(HEADER_STAMP_SECONDS))
      {
        group.getAttribute(HEADER_STAMP_SECONDS).write(time);
      }
      else
      {
        group.createAttribute(HEADER_STAMP_SECONDS, time);
      }
      m_file.flush();
      return;
    };

    case HighFive::ObjectType::Dataset:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::DataSet dataset = m_file.getDataSet(id);
      if (dataset.hasAttribute(HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(HEADER_STAMP_SECONDS).write(time);
      }
      else
      {
        dataset.createAttribute(HEADER_STAMP_SECONDS, time);
      }
      m_file.flush();
      return;
    };
    default:
      return;
  }
}

bool SeerepHDF5IO::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return hasTime(datatypeGroup, uuid + "/" + RAWDATA);
}

bool SeerepHDF5IO::hasTime(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file.exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file.getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file.getName());
  }

  switch (m_file.getObjectType(id))
  {
    case HighFive::ObjectType::Group:
      std::cout << "get group " << id << std::endl;
      return m_file.getGroup(id).hasAttribute(HEADER_STAMP_SECONDS);

    case HighFive::ObjectType::Dataset:
      std::cout << "get dataset " << id << std::endl;
      return m_file.getDataSet(id).hasAttribute(HEADER_STAMP_SECONDS);

    default:
      return false;
  }
}

void SeerepHDF5IO::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeled>& boundingboxLabeled)
{
  if (!boundingboxLabeled.empty())
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    for (auto label : boundingboxLabeled)
    {
      labels.push_back(label.label());
      std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                               label.boundingbox().point_min().z(), label.boundingbox().point_max().x(),
                               label.boundingbox().point_max().y(), label.boundingbox().point_max().z() };
      boundingBoxes.push_back(box);
    }

    HighFive::DataSet datasetLabels =
        m_file.createDataSet<std::string>(id + "/" + LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes =
        m_file.createDataSet<double>(id + "/" + LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    m_file.flush();
  }
}

void SeerepHDF5IO::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeled>& boundingbox2DLabeled)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!boundingbox2DLabeled.empty())
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    for (auto label : boundingbox2DLabeled)
    {
      labels.push_back(label.label());
      std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                               label.boundingbox().point_max().x(), label.boundingbox().point_max().y() };
      boundingBoxes.push_back(box);
    }

    HighFive::DataSet datasetLabels =
        m_file.createDataSet<std::string>(id + "/" + LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes =
        m_file.createDataSet<double>(id + "/" + LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    m_file.flush();
  }
}

std::optional<google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>>
SeerepHDF5IO::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file.exist(id + "/" + LABELBB))
  {
    std::cout << "id " << id + "/" + LABELBB << " does not exist in file " << m_file.getName() << std::endl;
    return std::nullopt;
  }
  if (!m_file.exist(id + "/" + LABELBBBOXES))
  {
    std::cout << "id " << id + "/" + LABELBBBOXES << " does not exist in file " << m_file.getName() << std::endl;
    return std::nullopt;
  }

  std::vector<std::string> labels;
  std::vector<std::vector<double>> boundingBoxes;

  HighFive::DataSet datasetLabels = m_file.getDataSet(id + "/" + LABELBB);
  datasetLabels.read(labels);

  HighFive::DataSet datasetBoxes = m_file.getDataSet(id + "/" + LABELBBBOXES);
  datasetBoxes.read(boundingBoxes);

  if (labels.size() != boundingBoxes.size())
  {
    std::cout << "size of labels (" << labels.size() << ") and size of bounding boxes (" << boundingBoxes.size()
              << ") do not fit." << std::endl;
    return std::nullopt;
  }

  google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled> result;

  for (int i = 0; i < labels.size(); i++)
  {
    seerep::BoundingBox2DLabeled bblabeled;
    bblabeled.set_label(labels.at(i));

    bblabeled.mutable_boundingbox()->mutable_point_min()->set_x(boundingBoxes.at(i).at(0));
    bblabeled.mutable_boundingbox()->mutable_point_min()->set_y(boundingBoxes.at(i).at(1));
    bblabeled.mutable_boundingbox()->mutable_point_max()->set_x(boundingBoxes.at(i).at(2));
    bblabeled.mutable_boundingbox()->mutable_point_max()->set_y(boundingBoxes.at(i).at(3));

    result.Add(std::move(bblabeled));
  }

  return result;
}

void SeerepHDF5IO::writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                      const google::protobuf::RepeatedPtrField<std::string>& labelsGeneral)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!labelsGeneral.empty())
  {
    std::vector<std::string> labels;
    for (auto label : labelsGeneral)
    {
      labels.push_back(label);
    }

    HighFive::DataSet datasetLabels =
        m_file.createDataSet<std::string>(id + "/" + LABELGENERAL, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    m_file.flush();
  }
}

std::optional<google::protobuf::RepeatedPtrField<std::string>>
SeerepHDF5IO::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file.exist(id + "/" + LABELGENERAL))
  {
    std::cout << "id " << id + "/" + LABELGENERAL << " does not exist in file " << m_file.getName() << std::endl;
    return std::nullopt;
  }

  std::vector<std::string> labels;
  HighFive::DataSet datasetLabels = m_file.getDataSet(id + "/" + LABELGENERAL);
  datasetLabels.read(labels);

  google::protobuf::RepeatedPtrField<std::string> result;

  for (int i = 0; i < labels.size(); i++)
  {
    result.Add(std::move(labels.at(i)));
  }

  return result;
}

void SeerepHDF5IO::writePointCloud2(const std::string& uuid, const seerep::PointCloud2& pointcloud2)
{
  std::string id = HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space({ pointcloud2.height(), pointcloud2.width(), 3 });

  if (!m_file.exist(id))
  {
    std::cout << "data id " << id << " does not exist! Creat new dataset in hdf5" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.createDataSet<float>(id, data_space));
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
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(id));
    data_set_ptr->getAttribute(HEIGHT).write(pointcloud2.height());
    data_set_ptr->getAttribute(WIDTH).write(pointcloud2.width());
    data_set_ptr->getAttribute(IS_BIGENDIAN).write(pointcloud2.is_bigendian());
    data_set_ptr->getAttribute(POINT_STEP).write(pointcloud2.point_step());
    data_set_ptr->getAttribute(ROW_STEP).write(pointcloud2.row_step());
    data_set_ptr->getAttribute(IS_DENSE).write(pointcloud2.is_dense());
  }

  writePointFieldAttributes(*data_set_ptr, pointcloud2.fields());

  const uint8_t* begin = reinterpret_cast<const uint8_t*>(pointcloud2.data().c_str());
  std::vector<std::vector<std::vector<float>>> point_data;
  point_data.resize(pointcloud2.height());

  seerep_hdf5::PointCloud2ConstIterator<float> x_iter(pointcloud2, "x");
  seerep_hdf5::PointCloud2ConstIterator<float> y_iter(pointcloud2, "y");
  seerep_hdf5::PointCloud2ConstIterator<float> z_iter(pointcloud2, "z");

  for (int i = 0; i < pointcloud2.height(); i++)
  {
    const uint8_t* row = begin + i * pointcloud2.row_step();
    point_data[i].reserve(pointcloud2.width());
    for (int y = 0; y < pointcloud2.width(); y++)
    {
      point_data[i].push_back(std::vector{ *x_iter, *y_iter, *z_iter });
      ++x_iter, ++y_iter, ++z_iter;
    }
  }

  data_set_ptr->write(point_data);

  writeHeaderAttributes(*data_set_ptr, pointcloud2.header());
  writeBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, uuid, pointcloud2.labels_bb());

  m_file.flush();
}

// TODO read partial point cloud, e.g. only xyz without color, etc.
std::optional<seerep::PointCloud2> SeerepHDF5IO::readPointCloud2(const std::string& id)
{
  if (!m_file.exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file.getName() << std::endl;
    return std::nullopt;
  }
  std::cout << "get Dataset" << std::endl;
  HighFive::DataSet data_set = m_file.getDataSet(id);

  seerep::PointCloud2 pointcloud2;

  std::cout << "read header attributes" << std::endl;
  *pointcloud2.mutable_header() = readHeaderAttributes(data_set);

  std::cout << "get attributes" << std::endl;
  uint32_t height, width, point_step, row_step;
  bool is_bigendian, is_dense;
  std::cout << "read height" << std::endl;
  data_set.getAttribute(HEIGHT).read(height);
  std::cout << "read width" << std::endl;
  data_set.getAttribute(WIDTH).read(width);
  std::cout << "read is_bigendian" << std::endl;
  data_set.getAttribute(IS_BIGENDIAN).read(is_bigendian);
  std::cout << "read point_step" << std::endl;
  data_set.getAttribute(POINT_STEP).read(point_step);
  std::cout << "read row_step" << std::endl;
  data_set.getAttribute(ROW_STEP).read(row_step);
  std::cout << "read is_dense" << std::endl;
  data_set.getAttribute(IS_DENSE).read(is_dense);

  pointcloud2.set_height(height);
  pointcloud2.set_width(width);
  pointcloud2.set_is_bigendian(is_bigendian);
  pointcloud2.set_point_step(point_step);
  pointcloud2.set_row_step(row_step);
  pointcloud2.set_is_dense(is_dense);

  std::cout << "read point field attributes" << std::endl;
  *pointcloud2.mutable_fields() = readPointFieldAttributes(data_set);

  std::cout << "read Dataset" << std::endl;

  seerep_hdf5::PointCloud2Iterator<float> x_iter(pointcloud2, "x");
  seerep_hdf5::PointCloud2Iterator<float> y_iter(pointcloud2, "y");
  seerep_hdf5::PointCloud2Iterator<float> z_iter(pointcloud2, "z");

  std::vector<std::vector<std::vector<float>>> read_data;
  data_set.read(read_data);

  for (auto column : read_data)
  {
    for (auto row : column)
    {
      *x_iter = row[0];
      *y_iter = row[1];
      *z_iter = row[2];
      ++x_iter, ++y_iter, ++z_iter;
    }
  }

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
  if (!m_file.exist(id))
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.createDataSet<uint8_t>(id, data_space));
  else
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(id));
  writePointAttributes(*data_set_ptr, point);
  m_file.flush();
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
  if (!m_file.exist(id))
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.createDataSet<uint8_t>(id, data_space));
  else
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(id));
  writeQuaternionAttributes(*data_set_ptr, quaternion);
  m_file.flush();
}

void SeerepHDF5IO::writePose(const std::string& id, const seerep::Pose& pose)
{
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space(0);
  if (!m_file.exist(id))
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.createDataSet<uint8_t>(id, data_space));
  else
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(id));
  writePointAttributes(*data_set_ptr, pose.position(), POSITION + "/");
  writeQuaternionAttributes(*data_set_ptr, pose.orientation(), ORIENTATION + "/");
  m_file.flush();
}

void SeerepHDF5IO::writePoseStamped(const std::string& id, const seerep::PoseStamped& pose)
{
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space(0);
  if (!m_file.exist(id))
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.createDataSet<uint8_t>(id, data_space));
  else
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(id));
  writeHeaderAttributes(*data_set_ptr, pose.header());
  writePointAttributes(*data_set_ptr, pose.pose().position(), POSE + "/" + POSITION + "/");
  writeQuaternionAttributes(*data_set_ptr, pose.pose().orientation(), POSE + "/" + ORIENTATION + "/");
  m_file.flush();
}

std::vector<std::string> SeerepHDF5IO::getGroupDatasets(const std::string& id)
{
  std::vector<std::string> rootObjects = m_file.listObjectNames();

  if (id.empty())
  {
    return rootObjects;
  }
  else
  {
    // check if rootObjects contains the group id
    if (std::find(rootObjects.begin(), rootObjects.end(), id) != rootObjects.end())
    {
      return m_file.getGroup(id).listObjectNames();
    }
    else
    {
      return std::vector<std::string>();
    }
  }
}

void SeerepHDF5IO::writeProjectname(const std::string& projectname)
{
  if (!m_file.hasAttribute(PROJECTNAME))
  {
    m_file.createAttribute<std::string>(PROJECTNAME, projectname);
  }
  else
  {
    m_file.getAttribute(PROJECTNAME).write(projectname);
  }
}

std::string SeerepHDF5IO::readProjectname()
{
  std::string projectname;
  if (m_file.hasAttribute(PROJECTNAME))
  {
    m_file.getAttribute(PROJECTNAME).read(projectname);
  }
  return projectname;
}

} /* namespace seerep_hdf5 */
