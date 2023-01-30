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

  // CloudInfo info = getCloudInfo(pointcloud2);

  // if (info.has_rgb)
  // {
  //   writeColorsRGB(uuid, pointcloud2);
  // }
  // if (info.has_rgba)
  // {
  //   writeColorsRGBA(uuid, pointcloud2);
  // }

  // // TODO normals
  // if (!info.other_fields.empty())
  // {
  //   writeOtherFields(uuid, pointcloud2, info.other_fields);
  // }

  m_file->flush();
}

void Hdf5PyPointCloud::writePoints(HighFive::Group& cloud_group, const std::string& cloud_group_id,
                                   std::map<std::string, bool>& processed,
                                   const std::map<std::string, py::array_t<float>>& channels)
{
  std::vector<std::vector<std::vector<float>>> point_data(0);

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  // find channels to use
  for (const auto& [name, data] : channels)
  {
    py::buffer_info buff_info = data.request();
    if (name.compare("xyz") == 0)
    {
      // directly use 'xyz' channel

      // TODO: check shape[1] for correct size
      point_data.resize(1);
      point_data[0].reserve(buff_info.shape[0]);
      for (unsigned int i = 0; i < buff_info.shape[0]; i++)
      {
        point_data[0].push_back(std::vector{ data.at(i, 0), data.at(i, 1), data.at(i, 2) });

        min[0] = std::min(data.at(i, 0), min[0]);
        min[1] = std::min(data.at(i, 1), min[1]);
        min[2] = std::min(data.at(i, 2), min[2]);

        max[0] = std::max(data.at(i, 0), max[0]);
        max[1] = std::max(data.at(i, 1), max[1]);
        max[2] = std::max(data.at(i, 2), max[2]);
      }

      processed["xyz"] = true;

      break;
    }
    else if (name.compare("x") == 0)
    {
      // construct 'xyz' channel from 'x', 'y' and 'z' channels

      const py::array_t<float>& data_x = data;

      auto search_y = channels.find("y");
      auto search_z = channels.find("z");

      if (search_y == channels.end() || search_z == channels.end())
      {
        throw std::invalid_argument(
            "you need to either specify 'x', 'y' and 'z' channels individually or a 'xyz' channel");
      }

      const py::array_t<float>& data_y = search_y->second;
      const py::array_t<float>& data_z = search_z->second;

      // TODO: check for correct shape of y and z channels
      point_data.resize(1);
      point_data[0].reserve(buff_info.shape[0]);
      for (unsigned int i = 0; i < buff_info.shape[0]; i++)
      {
        point_data[0].push_back(std::vector{ data_x.at(i), data_y.at(i), data_z.at(i) });

        min[0] = std::min(data_x.at(i), min[0]);
        min[1] = std::min(data_y.at(i), min[1]);
        min[2] = std::min(data_z.at(i), min[2]);

        max[0] = std::max(data_x.at(i), max[0]);
        max[1] = std::max(data_y.at(i), max[1]);
        max[2] = std::max(data_z.at(i), max[2]);
      }

      processed["x"] = true;
      processed["y"] = true;
      processed["z"] = true;

      break;
    }
  }

  if (point_data.size() == 0)
  {
    throw std::invalid_argument("you need to either specify 'x', 'y' and 'z' channels individually or a 'xyz' channel");
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
  for (const auto& [name, data] : channels)
  {
    py::buffer_info buff_info = data.request();
    if (name.compare("rgb") == 0)
    {
      // directly use 'rgb' channel

      // TODO: check shape[1] for correct size
      color_data.resize(1);
      color_data[0].reserve(buff_info.shape[0]);
      for (unsigned int i = 0; i < buff_info.shape[0]; i++)
      {
        color_data[0].push_back(std::vector{ data.at(i, 0), data.at(i, 1), data.at(i, 2) });
      }

      processed["rgb"] = true;

      break;
    }
    else if (name.compare("rgba") == 0)
    {
      // directly use 'rgba' channel

      // TODO: check shape[1] for correct size
      color_data.resize(1);
      color_data[0].reserve(buff_info.shape[0]);
      for (unsigned int i = 0; i < buff_info.shape[0]; i++)
      {
        color_data[0].push_back(std::vector{ data.at(i, 0), data.at(i, 1), data.at(i, 2), data.at(i, 3) });
      }

      processed["rgba"] = true;

      break;
    }
    else if (name.compare("r") == 0)
    {
      // construct color channel from individual component channels

      const py::array_t<float>& data_r = data;

      auto search_g = channels.find("g");
      auto search_b = channels.find("b");
      auto search_a = channels.find("a");

      if (search_g == channels.end() || search_b == channels.end())
      {
        continue;
      }

      const py::array_t<float>& data_g = search_g->second;
      const py::array_t<float>& data_b = search_b->second;

      if (search_a == channels.end())
      {
        // TODO: check for correct shape of g and b channels
        color_data.resize(1);
        color_data[0].reserve(buff_info.shape[0]);
        for (unsigned int i = 0; i < buff_info.shape[0]; i++)
        {
          color_data[0].push_back(std::vector{ data_r.at(i), data_g.at(i), data_b.at(i) });
        }
      }
      else
      {
        const py::array_t<float>& data_a = search_a->second;

        // TODO: check for correct shape of g, b and a channels
        color_data.resize(1);
        color_data[0].reserve(buff_info.shape[0]);
        for (unsigned int i = 0; i < buff_info.shape[0]; i++)
        {
          color_data[0].push_back(std::vector{ data_r.at(i), data_g.at(i), data_b.at(i), data_a.at(i) });
        }

        processed["a"] = true;
      }

      processed["r"] = true;
      processed["g"] = true;
      processed["b"] = true;

      break;
    }
  }

  if (color_data.size() == 0)
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
  for (const auto& [name, data] : channels)
  {
    py::buffer_info buff_info = data.request();
    if (name.compare("normal") == 0)
    {
      // directly use 'rgb' channel

      // TODO: check shape[1] for correct size
      normal_data.resize(1);
      normal_data[0].reserve(buff_info.shape[0]);
      for (unsigned int i = 0; i < buff_info.shape[0]; i++)
      {
        normal_data[0].push_back(std::vector{ data.at(i, 0), data.at(i, 1), data.at(i, 2) });
      }

      processed["normal"] = true;

      break;
    }
    else if (name.compare("nx") == 0)
    {
      // construct color channel from individual component channels

      const py::array_t<float>& data_nx = data;

      auto search_ny = channels.find("ny");
      auto search_nz = channels.find("nz");

      if (search_ny == channels.end() || search_nz == channels.end())
      {
        continue;
      }

      const py::array_t<float>& data_ny = search_ny->second;
      const py::array_t<float>& data_nz = search_nz->second;

      // TODO: check for correct shape of g and b channels
      normal_data.resize(1);
      normal_data[0].reserve(buff_info.shape[0]);
      for (unsigned int i = 0; i < buff_info.shape[0]; i++)
      {
        normal_data[0].push_back(std::vector{ data_nx.at(i), data_ny.at(i), data_nz.at(i) });
      }

      processed["nx"] = true;
      processed["ny"] = true;
      processed["nz"] = true;

      break;
    }
  }

  if (normal_data.size() == 0)
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

} /* namespace seerep_hdf5_py */
