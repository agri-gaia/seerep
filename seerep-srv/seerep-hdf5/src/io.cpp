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
  {
    dataSetPtr->createAttribute(attributeField, value);
  }
  else
  {
    dataSetPtr->getAttribute(attributeField).write(value);
  }
  m_file.flush();
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
    m_file.flush();
  }
}

template <class T>
void SeerepHDF5IO::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header)
{
  if (!object.hasAttribute(HEADER_STAMP_SECONDS))
    object.createAttribute(HEADER_STAMP_SECONDS, header.stamp().seconds());
  else
    object.getAttribute(HEADER_STAMP_SECONDS).write(header.stamp().seconds());

  if (!object.hasAttribute(HEADER_STAMP_NANOS))
    object.createAttribute(HEADER_STAMP_NANOS, header.stamp().nanos());
  else
    object.getAttribute(HEADER_STAMP_NANOS).write(header.stamp().nanos());

  if (!object.hasAttribute(HEADER_FRAME_ID))
    object.createAttribute(HEADER_FRAME_ID, header.frame_id());
  else
    object.getAttribute(HEADER_FRAME_ID).write(header.frame_id());

  if (!object.hasAttribute(HEADER_SEQ))
    object.createAttribute(HEADER_SEQ, header.seq());
  else
    object.getAttribute(HEADER_SEQ).write(header.seq());
}

template <class T>
seerep::Header SeerepHDF5IO::readHeaderAttributes(HighFive::AnnotateTraits<T>& object)
{
  seerep::Header header;

  int64_t seconds;
  int32_t nanos;
  uint32_t seq;

  object.getAttribute(HEADER_FRAME_ID).read(header.mutable_frame_id());

  object.getAttribute(HEADER_STAMP_SECONDS).read(seconds);
  object.getAttribute(HEADER_STAMP_NANOS).read(nanos);
  object.getAttribute(HEADER_SEQ).read(seq);

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
    HighFive::Group& cloud_group, const google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField)
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

  if (!cloud_group.hasAttribute(FIELD_NAME))
    cloud_group.createAttribute(FIELD_NAME, names);
  else
    cloud_group.getAttribute(FIELD_NAME).write(names);

  if (!cloud_group.hasAttribute(FIELD_OFFSET))
    cloud_group.createAttribute(FIELD_OFFSET, offsets);
  else
    cloud_group.getAttribute(FIELD_OFFSET).write(offsets);

  if (!cloud_group.hasAttribute(FIELD_DATATYPE))
    cloud_group.createAttribute(FIELD_DATATYPE, datatypes);
  else
    cloud_group.getAttribute(FIELD_DATATYPE).write(datatypes);

  if (!cloud_group.hasAttribute(FIELD_COUNT))
    cloud_group.createAttribute(FIELD_COUNT, counts);
  else
    cloud_group.getAttribute(FIELD_COUNT).write(counts);
}

google::protobuf::RepeatedPtrField<seerep::PointField>
SeerepHDF5IO::readPointFieldAttributes(HighFive::Group& cloud_group)
{
  google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField;

  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;

  cloud_group.getAttribute(FIELD_NAME).read(names);
  cloud_group.getAttribute(FIELD_OFFSET).read(offsets);
  cloud_group.getAttribute(FIELD_DATATYPE).read(datatypes);
  cloud_group.getAttribute(FIELD_COUNT).read(counts);

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

std::optional<std::string> SeerepHDF5IO::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  std::string hdf5DatasetRawDataPath = id + "/" + RAWDATA;
  if (!m_file.exist(hdf5DatasetRawDataPath))
  {
    std::cout << "id " << hdf5DatasetRawDataPath << " does not exist in file " << m_file.getName() << std::endl;
    throw std::invalid_argument("id " + hdf5DatasetRawDataPath + " does not exist in file " + m_file.getName());
  }
  std::cout << "get dataset " << hdf5DatasetRawDataPath << std::endl;
  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetRawDataPath));

  if (data_set_ptr->hasAttribute(HEADER_FRAME_ID))
  {
    std::string frameId;
    data_set_ptr->getAttribute(HEADER_FRAME_ID).read(frameId);
    return frameId;
  }
  else
  {
    return std::nullopt;
  }
}

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

void SeerepHDF5IO::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                   int64_t& nanos)
{
  readTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void SeerepHDF5IO::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos)
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
        group.getAttribute(HEADER_STAMP_SECONDS).read(secs);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_SECONDS);
      }
      if (group.hasAttribute(HEADER_STAMP_NANOS))
      {
        group.getAttribute(HEADER_STAMP_NANOS).read(nanos);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_NANOS);
      }
    };
    break;

    case HighFive::ObjectType::Dataset:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::DataSet dataset = m_file.getDataSet(id);
      if (dataset.hasAttribute(HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(HEADER_STAMP_SECONDS).read(secs);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_SECONDS);
      }
      if (dataset.hasAttribute(HEADER_STAMP_NANOS))
      {
        dataset.getAttribute(HEADER_STAMP_NANOS).read(nanos);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_NANOS);
      }
    };
    break;

    default:
      secs = std::numeric_limits<uint64_t>::min();
      nanos = std::numeric_limits<uint64_t>::min();
  }
}

void SeerepHDF5IO::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                  const int64_t& nanos)
{
  writeTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void SeerepHDF5IO::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                             const int64_t& nanos)
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
        group.getAttribute(HEADER_STAMP_SECONDS).write(secs);
      }
      else
      {
        group.createAttribute(HEADER_STAMP_SECONDS, secs);
      }
      if (group.hasAttribute(HEADER_STAMP_NANOS))
      {
        group.getAttribute(HEADER_STAMP_NANOS).write(nanos);
      }
      else
      {
        group.createAttribute(HEADER_STAMP_NANOS, nanos);
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
        dataset.getAttribute(HEADER_STAMP_SECONDS).write(secs);
      }
      else
      {
        dataset.createAttribute(HEADER_STAMP_SECONDS, secs);
      }
      if (dataset.hasAttribute(HEADER_STAMP_NANOS))
      {
        dataset.getAttribute(HEADER_STAMP_NANOS).write(nanos);
      }
      else
      {
        dataset.createAttribute(HEADER_STAMP_NANOS, nanos);
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
      return m_file.getGroup(id).hasAttribute(HEADER_STAMP_SECONDS) &&
             m_file.getGroup(id).hasAttribute(HEADER_STAMP_NANOS);

    case HighFive::ObjectType::Dataset:
      std::cout << "get dataset " << id << std::endl;
      return m_file.getDataSet(id).hasAttribute(HEADER_STAMP_SECONDS) &&
             m_file.getDataSet(id).hasAttribute(HEADER_STAMP_NANOS);

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
  std::string cloud_group_id = HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::Group> data_group_ptr;

  if (!m_file.exist(cloud_group_id))
  {
    std::cout << "data id " << cloud_group_id << " does not exist! Creat new dataset in hdf5" << std::endl;
    data_group_ptr = std::make_shared<HighFive::Group>(m_file.createGroup(cloud_group_id));
    data_group_ptr->createAttribute(HEIGHT, pointcloud2.height());
    data_group_ptr->createAttribute(WIDTH, pointcloud2.width());
    data_group_ptr->createAttribute(IS_BIGENDIAN, pointcloud2.is_bigendian());
    data_group_ptr->createAttribute(POINT_STEP, pointcloud2.point_step());
    data_group_ptr->createAttribute(ROW_STEP, pointcloud2.row_step());
    data_group_ptr->createAttribute(IS_DENSE, pointcloud2.is_dense());
  }
  else
  {
    std::cout << "data id " << cloud_group_id << " already exists!" << std::endl;
    data_group_ptr = std::make_shared<HighFive::Group>(m_file.getGroup(cloud_group_id));
    data_group_ptr->getAttribute(HEIGHT).write(pointcloud2.height());
    data_group_ptr->getAttribute(WIDTH).write(pointcloud2.width());
    data_group_ptr->getAttribute(IS_BIGENDIAN).write(pointcloud2.is_bigendian());
    data_group_ptr->getAttribute(POINT_STEP).write(pointcloud2.point_step());
    data_group_ptr->getAttribute(ROW_STEP).write(pointcloud2.row_step());
    data_group_ptr->getAttribute(IS_DENSE).write(pointcloud2.is_dense());
  }

  writePointFieldAttributes(*data_group_ptr, pointcloud2.fields());
  // TODO
  writeHeaderAttributes(*data_group_ptr, pointcloud2.header());

  std::string points_id = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/points";
  HighFive::DataSpace data_space({ pointcloud2.height(), pointcloud2.width(), 3 });

  std::shared_ptr<HighFive::DataSet> points_dataset_ptr;
  if (!m_file.exist(points_id))
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file.createDataSet<float>(points_id, data_space));
  else
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(points_id));

  std::vector<std::vector<std::vector<float>>> point_data;
  point_data.resize(pointcloud2.height());

  seerep_hdf5::PointCloud2ConstIterator<float> x_iter(pointcloud2, "x");
  seerep_hdf5::PointCloud2ConstIterator<float> y_iter(pointcloud2, "y");
  seerep_hdf5::PointCloud2ConstIterator<float> z_iter(pointcloud2, "z");

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  for (int i = 0; i < pointcloud2.height(); i++)
  {
    point_data[i].reserve(pointcloud2.width());
    for (int j = 0; j < pointcloud2.width(); j++)
    {
      const float& x = *x_iter;
      const float& y = *y_iter;
      const float& z = *z_iter;

      // compute bounding box
      if (x < min[0])
        min[0] = x;
      if (x > max[0])
        max[0] = x;

      if (y < min[1])
        min[1] = y;
      if (y > max[1])
        max[1] = y;

      if (z < min[2])
        min[2] = z;
      if (z > max[2])
        max[2] = z;

      point_data[i].push_back(std::vector{ x, y, z });

      ++x_iter, ++y_iter, ++z_iter;
    }
  }

  const std::vector boundingbox{ min[0], min[1], min[2], max[0], max[1], max[2] };
  if (!m_file.exist(cloud_group_id))
  {
    data_group_ptr->createAttribute(BOUNDINGBOX, boundingbox);
  }
  else
  {
    data_group_ptr->getAttribute(BOUNDINGBOX).write(boundingbox);
  }

  points_dataset_ptr->write(point_data);

  writeBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, uuid, pointcloud2.labels_bb());

  m_file.flush();
}

std::map<std::string, HighFive::Group> SeerepHDF5IO::getPointClouds()
{
  std::map<std::string, HighFive::Group> map;
  if (m_file.exist(HDF5_GROUP_POINTCLOUD))
  {
    const HighFive::Group& clouds_group = m_file.getGroup(HDF5_GROUP_POINTCLOUD);
    for (std::string& cloud_name : clouds_group.listObjectNames())
    {
      map.insert(std::pair(cloud_name, clouds_group.getGroup(cloud_name)));
    }
  }
  return map;
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

  // TODO
  //*pointcloud2.mutable_fields() = readPointFieldAttributes(data_set);

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
  m_file.flush();
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

void SeerepHDF5IO::writeProjectFrameId(const std::string& frameId)
{
  if (!m_file.hasAttribute(PROJECTFRAMEID))
  {
    m_file.createAttribute<std::string>(PROJECTFRAMEID, frameId);
  }
  else
  {
    m_file.getAttribute(PROJECTFRAMEID).write(frameId);
  }
  m_file.flush();
}

std::string SeerepHDF5IO::readProjectFrameId()
{
  std::string frameId;
  if (m_file.hasAttribute(PROJECTFRAMEID))
  {
    m_file.getAttribute(PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

void SeerepHDF5IO::writeTransformStamped(const seerep::TransformStamped& tf)
{
  std::string hdf5DatasetPath = HDF5_GROUP_TF + "/" + tf.header().frame_id() + "_" + tf.child_frame_id();
  std::string hdf5DatasetTimePath = hdf5DatasetPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5DatasetPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5DatasetPath + "/" + "rotation";

  std::shared_ptr<HighFive::DataSet> data_set_time_ptr, data_set_trans_ptr, data_set_rot_ptr;
  uint64_t size = 0;

  if (!m_file.exist(hdf5DatasetPath))
  {
    std::cout << "data id " << hdf5DatasetPath << " does not exist! Creat new dataset in hdf5" << std::endl;
    HighFive::Group group = m_file.createGroup(hdf5DatasetPath);
    group.createAttribute("CHILD_FRAME", tf.child_frame_id());
    group.createAttribute("PARENT_FRAME", tf.header().frame_id());

    // TIME
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_time({ 1, 2 }, { HighFive::DataSpace::UNLIMITED, 2 });
    // Use chunking
    HighFive::DataSetCreateProps props_time;
    props_time.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 2 }));
    data_set_time_ptr = std::make_shared<HighFive::DataSet>(
        m_file.createDataSet<int64_t>(hdf5DatasetTimePath, data_space_time, props_time));

    // TRANSLATION
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_trans({ 1, 3 }, { HighFive::DataSpace::UNLIMITED, 3 });
    // Use chunking
    HighFive::DataSetCreateProps props_trans;
    props_trans.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 3 }));
    data_set_trans_ptr = std::make_shared<HighFive::DataSet>(
        m_file.createDataSet<double>(hdf5DatasetTransPath, data_space_trans, props_trans));

    // ROTATION
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_rot({ 1, 4 }, { HighFive::DataSpace::UNLIMITED, 4 });
    // Use chunking
    HighFive::DataSetCreateProps props_rot;
    props_rot.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 4 }));
    data_set_rot_ptr = std::make_shared<HighFive::DataSet>(
        m_file.createDataSet<double>(hdf5DatasetRotPath, data_space_rot, props_rot));
  }
  else
  {
    std::cout << "data id " << hdf5DatasetPath << " already exists!" << std::endl;
    data_set_time_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetTimePath));
    data_set_trans_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetTransPath));
    data_set_rot_ptr = std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetRotPath));

    HighFive::Group group = m_file.getGroup(hdf5DatasetPath);
    group.getAttribute(SIZE).read(size);

    // Resize the dataset to a larger size
    data_set_time_ptr->resize({ size + 1, 2 });
    data_set_trans_ptr->resize({ size + 1, 3 });
    data_set_rot_ptr->resize({ size + 1, 4 });
  }

  // write time
  std::vector<int64_t> time;
  time.push_back(tf.header().stamp().seconds());
  time.push_back(tf.header().stamp().nanos());
  data_set_time_ptr->select({ size, 0 }, { 1, 2 }).write(time);

  // write translation
  std::vector<double> trans;
  trans.push_back(tf.transform().translation().x());
  trans.push_back(tf.transform().translation().y());
  trans.push_back(tf.transform().translation().z());
  data_set_trans_ptr->select({ size, 0 }, { 1, 3 }).write(trans);

  // write rotation
  std::vector<double> rot;
  rot.push_back(tf.transform().rotation().x());
  rot.push_back(tf.transform().rotation().y());
  rot.push_back(tf.transform().rotation().z());
  rot.push_back(tf.transform().rotation().w());
  data_set_rot_ptr->select({ size, 0 }, { 1, 4 }).write(rot);

  // write the size as group attribute
  HighFive::Group group = m_file.getGroup(hdf5DatasetPath);
  if (!group.hasAttribute(SIZE))
    group.createAttribute(SIZE, ++size);
  else
    group.getAttribute(SIZE).write(++size);

  m_file.flush();
}

std::optional<std::vector<seerep::TransformStamped>> SeerepHDF5IO::readTransformStamped(const std::string& id)
{
  std::string hdf5GroupPath = HDF5_GROUP_TF + "/" + id;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  if (!m_file.exist(hdf5GroupPath) || !m_file.exist(hdf5DatasetTimePath) || !m_file.exist(hdf5DatasetTransPath) ||
      !m_file.exist(hdf5DatasetRotPath))
  {
    return std::nullopt;
  }

  std::cout << "loading " << hdf5GroupPath << std::endl;

  // read size
  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file.getGroup(hdf5GroupPath));
  int size;
  group_ptr->getAttribute(SIZE).read(size);
  if (size == 0)
  {
    std::cout << "tf data has size 0." << std::endl;
    return std::nullopt;
  }

  // read frames
  std::string parentframe;
  group_ptr->getAttribute("PARENT_FRAME").read(parentframe);
  std::string childframe;
  group_ptr->getAttribute("CHILD_FRAME").read(childframe);

  // read time
  std::shared_ptr<HighFive::DataSet> data_set_time_ptr =
      std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetTimePath));
  std::vector<std::vector<int64_t>> time;
  data_set_time_ptr->read(time);

  // read translation
  std::shared_ptr<HighFive::DataSet> data_set_trans_ptr =
      std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetTransPath));
  std::vector<std::vector<double>> trans;
  data_set_trans_ptr->read(trans);

  // read rotation
  std::shared_ptr<HighFive::DataSet> data_set_rot_ptr =
      std::make_shared<HighFive::DataSet>(m_file.getDataSet(hdf5DatasetRotPath));
  std::vector<std::vector<double>> rot;
  data_set_rot_ptr->read(rot);

  // check if all have the right size
  if (time.size() != size || trans.size() != size || rot.size() != size)
  {
    std::cout << "sizes of time (" << time.size() << "), translation (" << trans.size() << ") and rotation ("
              << rot.size() << ") not matching. Size expected by value in metadata (" << size << ")" << std::endl;
    return std::nullopt;
  }

  std::vector<seerep::TransformStamped> tfs;
  for (int i = 0; i < size; i++)
  {
    seerep::TransformStamped tf;
    tf.mutable_header()->set_frame_id(parentframe);
    tf.set_child_frame_id(childframe);

    tf.mutable_header()->mutable_stamp()->set_seconds(time.at(i).at(0));
    tf.mutable_header()->mutable_stamp()->set_nanos(time.at(i).at(1));

    seerep::Vector3 translation;
    translation.set_x(trans.at(i).at(0));
    translation.set_y(trans.at(i).at(1));
    translation.set_z(trans.at(i).at(2));
    *tf.mutable_transform()->mutable_translation() = translation;

    seerep::Quaternion rotation;
    rotation.set_x(rot.at(i).at(0));
    rotation.set_y(rot.at(i).at(1));
    rotation.set_z(rot.at(i).at(2));
    rotation.set_w(rot.at(i).at(3));
    *tf.mutable_transform()->mutable_rotation() = rotation;

    tfs.push_back(tf);
  }
  return tfs;
}

std::optional<std::vector<std::string>> SeerepHDF5IO::readTransformStampedFrames(const std::string& id)
{
  std::string hdf5GroupPath = HDF5_GROUP_TF + "/" + id;

  if (!m_file.exist(hdf5GroupPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file.getGroup(hdf5GroupPath));

  std::cout << "loading parent frame of " << hdf5GroupPath << std::endl;

  // read frames
  std::string parentframe;
  group_ptr->getAttribute("PARENT_FRAME").read(parentframe);
  std::string childframe;
  group_ptr->getAttribute("CHILD_FRAME").read(childframe);

  return std::vector<std::string>{ parentframe, childframe };
}

} /* namespace seerep_hdf5 */
