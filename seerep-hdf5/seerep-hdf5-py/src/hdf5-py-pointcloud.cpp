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
                                       const std::map<std::string, py::array_t<float>> channels)
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
    names.push_back(name);
    offsets.push_back(offset);
    datatypes.push_back(7);  // float32
    counts.push_back(1);

    offset += sizeof(float);
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
  writePoints(*data_group_ptr, cloud_group_id, channel_processed, channels);
  writeColors(cloud_group_id, channel_processed, channels);
  writeNormals(cloud_group_id, channel_processed, channels);

  for (const auto& [name, data] : channels)
  {
    if (!channel_processed[name])
    {
      writeOther(cloud_group_id, channel_processed, name, data);
    }
  }

  m_file->flush();
}

bool Hdf5PyPointCloud::getChannelData(const std::vector<std::string>& channel_names,
                                      const std::map<std::string, py::array_t<float>>& channels,
                                      std::map<std::string, bool>& processed,
                                      std::vector<std::vector<std::vector<float>>>& channel_data)
{
  // make sure that all channels exist
  for (const auto& channel_name : channel_names)
  {
    auto search = channels.find(channel_name);
    if (search == channels.end())
    {
      return false;
    }
  }

  // combine channels into single channel
  channel_data.clear();
  channel_data.resize(1);
  for (const auto& channel_name : channel_names)
  {
    auto search = channels.find(channel_name);
    const py::array_t<float>& data = search->second;

    py::buffer_info buff_info = data.request();
    if (channel_data[0].empty())
    {
      // reserve space for first channel
      channel_data[0].resize(buff_info.shape[0]);
    }
    else if ((std::size_t)buff_info.shape[0] != channel_data[0].size())
    {
      // not matching channel sizes
      channel_data.clear();
      return false;
    }

    for (unsigned int i = 0; i < buff_info.shape[0]; i++)
    {
      if (buff_info.shape.size() > 1)
      {
        for (unsigned int j = 0; j < buff_info.shape[1]; j++)
        {
          channel_data[0][i].push_back(data.at(i, j));
        }
      }
      else
      {
        channel_data[0][i].push_back(data.at(i));
      }
    }
  }

  // mark channels as processed
  for (const auto& channel_name : channel_names)
  {
    processed[channel_name] = true;
  }

  return true;
}

void Hdf5PyPointCloud::writePoints(HighFive::Group& cloud_group, const std::string& cloud_group_id,
                                   std::map<std::string, bool>& processed,
                                   const std::map<std::string, py::array_t<float>>& channels)
{
  std::vector<std::vector<std::vector<float>>> point_data(0);

  // find channels to use
  bool found_channels = false;
  std::vector<std::string> channels_names({ "xyz" });
  found_channels = getChannelData(channels_names, channels, processed, point_data);

  if (!found_channels)
  {
    channels_names = std::vector<std::string>{ "x", "y", "z" };
    found_channels = getChannelData(channels_names, channels, processed, point_data);
  }

  if (!found_channels)
  {
    // no channels found
    throw std::invalid_argument("you need to either specify 'x', 'y' and 'z' channels individually or a 'xyz' channel");
  }

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  for (const auto& point : point_data[0])
  {
    min[0] = std::min(point[0], min[0]);
    min[1] = std::min(point[1], min[1]);
    min[2] = std::min(point[2], min[2]);

    max[0] = std::max(point[0], max[0]);
    max[1] = std::max(point[1], max[1]);
    max[2] = std::max(point[2], max[2]);
  }

  // create dataset
  std::string points_id = cloud_group_id + "/points";
  HighFive::DataSpace data_space({ point_data.size(), point_data[0].size(), 3 });

  std::shared_ptr<HighFive::DataSet> points_dataset_ptr;
  if (!m_file->exist(points_id))
  {
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(points_id, data_space));
  }
  else
  {
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(points_id));
  }

  // write bounding box as attribute to dataset
  const std::vector<float> boundingbox{ min[0], min[1], min[2], max[0], max[1], max[2] };

  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(
      cloud_group, seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX, boundingbox);

  // write data to dataset
  points_dataset_ptr->write(point_data);
}

void Hdf5PyPointCloud::writeColors(const std::string& cloud_group_id, std::map<std::string, bool>& processed,
                                   const std::map<std::string, py::array_t<float>>& channels)
{
  std::vector<std::vector<std::vector<float>>> color_data(0);

  // find channels to use
  bool found_channels = false;
  std::vector<std::string> channels_names({ "rgba" });
  found_channels = getChannelData(channels_names, channels, processed, color_data);

  if (!found_channels)
  {
    channels_names = std::vector<std::string>{ "r", "g", "b", "a" };
    found_channels = getChannelData(channels_names, channels, processed, color_data);
  }

  if (!found_channels)
  {
    channels_names = std::vector<std::string>{ "rgb" };
    found_channels = getChannelData(channels_names, channels, processed, color_data);
  }

  if (!found_channels)
  {
    channels_names = std::vector<std::string>{ "r", "g", "b" };
    found_channels = getChannelData(channels_names, channels, processed, color_data);
  }

  if (!found_channels)
  {
    // no channels found
    return;
  }

  // create dataset
  const std::string colors_id = cloud_group_id + "/colors";
  HighFive::DataSpace data_space({ color_data.size(), color_data[0].size(), color_data[0][0].size() });

  std::shared_ptr<HighFive::DataSet> colors_dataset_ptr;
  if (!m_file->exist(colors_id))
  {
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(colors_id, data_space));
  }
  else
  {
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(colors_id));
  }

  // write data
  colors_dataset_ptr->write(color_data);
}

void Hdf5PyPointCloud::writeNormals(const std::string& cloud_group_id, std::map<std::string, bool>& processed,
                                    const std::map<std::string, py::array_t<float>>& channels)
{
  std::vector<std::vector<std::vector<float>>> normal_data(0);

  // find channels to use
  bool found_channels = false;
  std::vector<std::string> channels_names({ "normal" });
  found_channels = getChannelData(channels_names, channels, processed, normal_data);

  if (!found_channels)
  {
    channels_names = std::vector<std::string>{ "nx", "ny", "nz" };
    found_channels = getChannelData(channels_names, channels, processed, normal_data);
  }

  if (!found_channels)
  {
    // no channels found
    return;
  }

  // create dataset
  const std::string colors_id = cloud_group_id + "/normal";
  HighFive::DataSpace data_space({ normal_data.size(), normal_data[0].size(), normal_data[0][0].size() });

  std::shared_ptr<HighFive::DataSet> colors_dataset_ptr;
  if (!m_file->exist(colors_id))
  {
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(colors_id, data_space));
  }
  else
  {
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(colors_id));
  }

  // write data
  colors_dataset_ptr->write(normal_data);
}

void Hdf5PyPointCloud::writeOther(const std::string& cloud_group_id, std::map<std::string, bool>& processed,
                                  const std::string& channel_name, const py::array_t<float>& channel_data)
{
  std::vector<std::vector<std::vector<float>>> data(0);

  data.resize(1);
  py::buffer_info buff_info = channel_data.request();
  data[0].resize(buff_info.shape[0]);
  for (unsigned int i = 0; i < buff_info.shape[0]; i++)
  {
    if (buff_info.shape.size() > 1)
    {
      data[0][i].reserve(buff_info.shape[1]);
      for (unsigned int j = 0; j < buff_info.shape[1]; j++)
      {
        data[0][i].push_back(channel_data.at(i, j));
      }
    }
    else
    {
      data[0][i] = std::vector{ channel_data.at(i) };
    }
  }

  processed[channel_name] = true;

  // create dataset
  const std::string dataset_id = cloud_group_id + "/" + channel_name;
  HighFive::DataSpace data_space({ data.size(), data[0].size(), data[0][0].size() });

  std::shared_ptr<HighFive::DataSet> colors_dataset_ptr;
  if (!m_file->exist(dataset_id))
  {
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(dataset_id, data_space));
  }
  else
  {
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(dataset_id));
  }

  // write data
  colors_dataset_ptr->write(data);
}

} /* namespace seerep_hdf5_py */
