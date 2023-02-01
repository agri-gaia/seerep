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
                                       const std::map<std::string, py::array> channels)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string cloud_group_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::Group> data_group_ptr;

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

  // write basic attributes

  std::size_t point_step = channels.size() * sizeof(float);
  if (!m_file->exist(cloud_group_id))
  {
    data_group_ptr = std::make_shared<HighFive::Group>(m_file->createGroup(cloud_group_id));
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT, 1);
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH, num_points);
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN, false);
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP, point_step);
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP, num_points * point_step);
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE, true);
  }
  else
  {
    data_group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(cloud_group_id));
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT).write(1);
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH).write(num_points);
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN).write(false);
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP).write(point_step);
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP).write(num_points * point_step);
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE).write(true);
  }

  // write point fields

  std::vector<std::string> names(channels.size());
  std::vector<uint32_t> offsets(channels.size());
  std::vector<uint8_t> datatypes(channels.size());
  std::vector<uint32_t> counts(channels.size());
  uint32_t offset = 0;
  for (const auto& [name, data] : channels)
  {
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

    if (channel_info.shape.size() > 1)
    {
      counts.push_back(channel_info.shape[1]);
    }
    else
    {
      counts.push_back(1);
    }

    offset += data.dtype().itemsize();
  }

  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, names);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET, offsets);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(
      *data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE, datatypes);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, counts);

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

  // writeLabelsGeneral(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, uuid, pointcloud2.labels_general());
  // writeBoundingBoxLabeled(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, uuid, pointcloud2.labels_bb());

  // write data

  std::map<std::string, bool> channel_processed;
  for (const auto& [name, data] : channels)
  {
    channel_processed.insert({ name, false });
  }
  writePoints(cloud_group_id, channel_processed, channels);
  writeColors(cloud_group_id, channel_processed, channels);
  writeNormals(cloud_group_id, channel_processed, channels);

  for (const auto& [name, data] : channels)
  {
    if (!channel_processed[name])
    {
      writeOther(cloud_group_id, channel_processed, name, channels);
    }
  }

  m_file->flush();
}

void Hdf5PyPointCloud::writePoints(const std::string& cloud_group_id, std::map<std::string, bool>& processed,
                                   const std::map<std::string, py::array>& channels)
{
  std::vector<std::vector<std::string>> channel_names({ { "xyz" }, { "x", "y", "z" } });
  if (!writeChannelTyped<uint8_t, uint16_t, float, double>(cloud_group_id, "points", channel_names, processed, channels,
                                                           true))
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
