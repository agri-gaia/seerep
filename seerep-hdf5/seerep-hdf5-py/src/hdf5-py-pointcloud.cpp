#include "seerep-hdf5-py/hdf5-py-pointcloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

Hdf5PyPointCloud::Hdf5PyPointCloud(Hdf5FileWrapper& hdf5_file)
  : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex())
  // , Hdf5CorePointCloud(hdf5_file.getFile(), hdf5_file.getMutex())
  , Hdf5PyGeneral(hdf5_file)
{
}

std::vector<std::string> Hdf5PyPointCloud::getPointClouds()
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->exist(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD))
  {
    return std::vector<std::string>();
  }
  const HighFive::Group& clouds_group = m_file->getGroup(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD);
  return clouds_group.listObjectNames();
}

void Hdf5PyPointCloud::writePointCloud(const std::string& uuid, const std::string& frame_id, int64_t seconds,
                                       int32_t nanos, uint32_t sequence,
                                       const std::map<std::string, py::array> channels,
                                       const std::vector<GeneralLabel>& general_labels,
                                       const std::vector<CategorizedBoundingBoxLabel<3>>& bb_labels)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string cloud_group_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::Group> data_group_ptr = getHdf5Group(cloud_group_id);

  std::size_t num_points = 0;
  for (const auto& [name, data] : channels)
  {
    py::buffer_info buff_info = data.request();
    if (name.compare("xyz") == 0)
    {
      num_points = buff_info.shape[0];
      break;
    }
    else if (name.compare("x") == 0)
    {
      num_points = buff_info.shape[0];
      break;
    }
  }

  // write header

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, seconds);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).write(seconds);
  }

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, nanos);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).write(nanos);
  }

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, frame_id);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID).write(frame_id);
  }

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, sequence);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ).write(sequence);
  }

  // write labels
  Hdf5PyGeneral::writeBoundingBoxLabeled(cloud_group_id, bb_labels);
  Hdf5PyGeneral::writeLabelsGeneral(cloud_group_id, general_labels);

  // write data

  std::map<std::string, bool> channel_processed;
  for (const auto& [name, data] : channels)
  {
    channel_processed.insert({ name, false });
  }
  // writePoints(cloud_group_id, channel_processed, channels);
  // writeColors(cloud_group_id, channel_processed, channels);
  // writeNormals(cloud_group_id, channel_processed, channels);
  // TODO: re-add bounding box calculation and writing

  for (const auto& [name, data] : channels)
  {
    if (!channel_processed[name])
    {
      writeOther(cloud_group_id, channel_processed, name, channels);
    }
  }

  // write point fields

  std::vector<std::string> names(channels.size());
  std::vector<uint32_t> offsets(channels.size());
  std::vector<uint8_t> datatypes(channels.size());
  std::vector<uint32_t> counts(channels.size());
  uint32_t offset = 0;
  for (const auto& [name, data] : channels)
  {
    if (!channel_processed[name])
    {
      continue;
    }

    py::buffer_info channel_info = data.request();

    names.push_back(name);
    offsets.push_back(offset);

    switch (data.dtype().char_())
    {
      case 'b':
        datatypes.push_back(1);  // int8
        break;
      case 'i':
        datatypes.push_back(5);  // int32
        break;
      case 'B':
        datatypes.push_back(2);  // uint8
        break;
      case 'I':
        datatypes.push_back(6);  // uint32
        break;
      case 'f':
        datatypes.push_back(7);  // float32
        break;
      case 'd':
        datatypes.push_back(8);  // float64
        break;
    }

    uint32_t num_elements = 1;
    if (channel_info.shape.size() > 1)
    {
      num_elements = channel_info.shape[1];
    }
    counts.push_back(num_elements);

    offset += data.dtype().itemsize() * num_elements;
  }

  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, names);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET, offsets);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(
      *data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE, datatypes);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, counts);

  // write basic attributes

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT, 1);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT).write(1);
  }

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH, num_points);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH).write(num_points);
  }

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN, false);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN).write(false);
  }

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP, offset);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP).write(offset);
  }

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP, num_points * offset);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP).write(num_points * offset);
  }

  if (!data_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE))
  {
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE, true);
  }
  else
  {
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE).write(true);
  }

  m_file->flush();
}

std::map<std::string, py::array> Hdf5PyPointCloud::readPointCloud(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string cloud_group_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;

  if (!m_file->exist(cloud_group_id))
  {
    return std::map<std::string, py::array>();
  }

  HighFive::Group cloud_group = m_file->getGroup(cloud_group_id);

  uint32_t height, width, point_step, row_step;
  bool is_bigendian, is_dense;
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT).read(height);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH).read(width);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN).read(is_bigendian);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP).read(point_step);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP).read(row_step);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE).read(is_dense);

  std::vector<std::string> names;
  std::vector<uint32_t> offsets;
  std::vector<uint32_t> counts;
  std::vector<uint8_t> datatypes;

  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME).read(names);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET).read(offsets);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE).read(datatypes);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT).read(counts);

  if (names.size() != offsets.size() || names.size() != counts.size() || names.size() != datatypes.size())
  {
    throw std::invalid_argument("point field sizes do not match up for pointcloud " + uuid);
  }

  std::map<std::string, py::array> channels;

  for (std::size_t i = 0; i < names.size(); i++)
  {
    if (counts[i] == 0)
    {
      continue;
    }

    if (!m_file->exist(cloud_group_id + "/" + names[i]))
    {
      throw std::invalid_argument("pointcloud field " + names[i] + " not found in pointcloud " + uuid);
    }

    std::shared_ptr<HighFive::DataSet> dataset = getHdf5DataSet(cloud_group_id + "/" + names[i]);

    if (datatypes[i] == 1)
    {
      channels[names[i]] = readField<int8_t>(dataset);
    }
    else if (datatypes[i] == 5)
    {
      channels[names[i]] = readField<int32_t>(dataset);
    }
    else if (datatypes[i] == 2)
    {
      channels[names[i]] = readField<uint8_t>(dataset);
    }
    else if (datatypes[i] == 6)
    {
      channels[names[i]] = readField<uint32_t>(dataset);
    }
    else if (datatypes[i] == 7)
    {
      channels[names[i]] = readField<float>(dataset);
    }
    else if (datatypes[i] == 7)
    {
      channels[names[i]] = readField<double>(dataset);
    }
    else
    {
      throw std::invalid_argument("unknown type in field " + names[i] + " of pointcloud " + uuid);
    }
  }

  // TODO: add label reading

  return channels;
}

void Hdf5PyPointCloud::writePoints(const std::string& cloud_group_id, std::map<std::string, bool>& processed,
                                   const std::map<std::string, py::array>& channels)
{
  std::vector<std::vector<std::string>> channel_names({ { "xyz" }, { "x", "y", "z" } });
  if (!writeChannelTyped<float, double>(cloud_group_id, "points", channel_names, processed, channels, true))
  {
    // no channels found
    throw std::invalid_argument("you need to either specify 'x', 'y' and 'z' channels individually or a 'xyz' channel");
  }
}

void Hdf5PyPointCloud::writeColors(const std::string& cloud_group_id, std::map<std::string, bool>& processed,
                                   const std::map<std::string, py::array>& channels)
{
  std::vector<std::vector<std::string>> channel_names(
      { { "rgba" }, { "rgb" }, { "r", "g", "b" }, { "r", "g", "b", "a" } });
  writeChannelTyped<uint8_t, uint16_t, float, double>(cloud_group_id, "colors", channel_names, processed, channels,
                                                      false);
}

void Hdf5PyPointCloud::writeNormals(const std::string& cloud_group_id, std::map<std::string, bool>& processed,
                                    const std::map<std::string, py::array>& channels)
{
  std::vector<std::vector<std::string>> channel_names({ { "normal" }, { "nx", "ny", "nz" } });
  writeChannelTyped<float, double>(cloud_group_id, "normals", channel_names, processed, channels, false);
}

void Hdf5PyPointCloud::writeOther(const std::string& cloud_group_id, std::map<std::string, bool>& processed,
                                  const std::string& channel_name, const std::map<std::string, py::array>& channels)
{
  std::vector<std::vector<std::string>> channel_names({ { channel_name } });

  auto search = channels.find(channel_name);
  switch (search->second.dtype().char_())
  {
    case 'b':
      writeChannelTyped<char>(cloud_group_id, channel_name, channel_names, processed, channels, false);
      break;

    case 'i':
      writeChannelTyped<int>(cloud_group_id, channel_name, channel_names, processed, channels, false);
      break;

    case 'B':
      writeChannelTyped<uint8_t>(cloud_group_id, channel_name, channel_names, processed, channels, false);
      break;

    case 'I':
      writeChannelTyped<unsigned int>(cloud_group_id, channel_name, channel_names, processed, channels, false);
      break;

    case 'f':
      writeChannelTyped<float>(cloud_group_id, channel_name, channel_names, processed, channels, false);
      break;

    case 'd':
      writeChannelTyped<double>(cloud_group_id, channel_name, channel_names, processed, channels, false);
      break;
  }
}

} /* namespace seerep_hdf5_py */
