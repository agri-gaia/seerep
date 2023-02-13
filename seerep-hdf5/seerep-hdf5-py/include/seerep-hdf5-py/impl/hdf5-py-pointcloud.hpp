namespace seerep_hdf5_py
{

template <typename T>
bool Hdf5PyPointCloud::checkType(const py::dtype& type)
{
  // compare dtype with templated type T
  switch (type.char_())
  {
    case 'b':
      if (!std::is_same<T, char>::value)
      {
        return false;
      }
      break;

    case 'h':
      if (!std::is_same<T, short>::value)
      {
        return false;
      }
      break;

    case 'i':
      if (!std::is_same<T, int>::value)
      {
        return false;
      }
      break;

    case 'l':
      if (!std::is_same<T, long>::value)
      {
        return false;
      }
      break;

    case 'q':
      if (!std::is_same<T, long long>::value)
      {
        return false;
      }
      break;

    case 'B':
      if (!std::is_same<T, unsigned char>::value)
      {
        return false;
      }
      break;

    case 'H':
      if (!std::is_same<T, unsigned short>::value)
      {
        return false;
      }
      break;

    case 'I':
      if (!std::is_same<T, unsigned int>::value)
      {
        return false;
      }
      break;

    case 'L':
      if (!std::is_same<T, unsigned long>::value)
      {
        return false;
      }
      break;

    case 'Q':
      if (!std::is_same<T, unsigned long long>::value)
      {
        return false;
      }
      break;

    case 'f':
      if (!std::is_same<T, float>::value)
      {
        return false;
      }
      break;

    case 'd':
      if (!std::is_same<T, double>::value)
      {
        return false;
      }
      break;

    case 'g':
      if (!std::is_same<T, long double>::value)
      {
        return false;
      }
      break;
  }

  return true;
}

template <typename T>
bool Hdf5PyPointCloud::getChannelData(const std::vector<std::string>& channel_names,
                                      const std::map<std::string, py::array>& channels,
                                      std::vector<std::vector<std::vector<T>>>& channel_data)
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
    py::buffer_info buff_info = search->second.request();

    // check type
    if (!checkType<T>(search->second.dtype()))
    {
      channel_data.clear();
      return false;
    }

    // cast data for easy access
    const py::array_t<T>& data = static_cast<py::array_t<T>>(search->second);

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

  return true;
}

template <typename T, int Nchannels>
void Hdf5PyPointCloud::getMinMax(const std::vector<std::vector<std::vector<T>>>& data, std::array<T, Nchannels>& min,
                                 std::array<T, Nchannels>& max)
{
  for (std::size_t i = 0; i < Nchannels; i++)
  {
    min[i] = std::numeric_limits<T>::max();
    max[i] = std::numeric_limits<T>::min();

    for (const auto& dat : data[0])
    {
      min[i] = std::min(dat[i], min[i]);
      max[i] = std::max(dat[i], max[i]);
    }
  }
}

template <typename T, int Nchannels>
bool Hdf5PyPointCloud::writeBoundingBox(const std::string& cloud_group_id,
                                        const std::vector<std::string>& channel_names,
                                        const std::map<std::string, py::array>& channels)
{
  std::vector<std::vector<std::vector<T>>> channel_data(0);

  // find channels to use
  getChannelData<T>(channel_names, channels, channel_data);

  if (channel_data.size() == 0)
  {
    // no channels found
    return false;
  }

  std::array<T, Nchannels> min, max;
  getMinMax<T, Nchannels>(channel_data, min, max);

  std::shared_ptr<HighFive::Group> cloud_group = std::make_shared<HighFive::Group>(m_file->getGroup(cloud_group_id));

  // write bounding box as attribute to dataset
  std::vector<T> boundingbox(2 * Nchannels);
  for (std::size_t i = 0; i < Nchannels; i++)
  {
    boundingbox[i] = min[i];
    boundingbox[i + Nchannels] = max[i];
  }

  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(
      *cloud_group, seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX, boundingbox);

  return true;
}

template <typename T>
bool Hdf5PyPointCloud::writeChannelTyped(const std::string& cloud_group_id, const std::string& channel_dataset_id,
                                         const std::string& channel_name,
                                         const std::map<std::string, py::array>& channels)
{
  std::vector<std::vector<std::vector<T>>> channel_data(0);

  // find channels to use
  getChannelData<T>({ channel_name }, channels, channel_data);

  if (channel_data.size() == 0)
  {
    // no channels found
    return false;
  }

  // create dataset
  const std::string dataset_id = cloud_group_id + "/" + channel_dataset_id;
  HighFive::DataSpace data_space({ channel_data.size(), channel_data[0].size(), channel_data[0][0].size() });

  std::shared_ptr<HighFive::DataSet> dataset_ptr;
  if (!m_file->exist(dataset_id))
  {
    dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<T>(dataset_id, data_space));
  }
  else
  {
    dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(dataset_id));
  }

  // write data
  dataset_ptr->write(channel_data);

  return true;
}

template <typename T, typename Second, typename... Other>
bool Hdf5PyPointCloud::writeChannelTyped(const std::string& cloud_group_id, const std::string& channel_dataset_id,
                                         const std::string& channel_name,
                                         const std::map<std::string, py::array>& channels)
{
  if (writeChannelTyped<T>(cloud_group_id, channel_dataset_id, channel_name, channels))
  {
    return true;
  }

  return writeChannelTyped<Second, Other...>(cloud_group_id, channel_dataset_id, channel_name, channels);
}

template <typename T>
py::array Hdf5PyPointCloud::readField(std::shared_ptr<HighFive::DataSet> field_dataset)
{
  std::vector<std::size_t> dataset_dimensions = field_dataset->getSpace().getDimensions();
  std::cout << dataset_dimensions[0] << " " << dataset_dimensions[1] << " " << dataset_dimensions[2] << std::endl;

  // ignore hight for now -> organized pointclouds unsupported
  py::array field = py::array_t<T>({ dataset_dimensions[0] * dataset_dimensions[1], dataset_dimensions[2] });

  std::vector<std::vector<std::vector<T>>> hdf5_data;
  field_dataset->read(hdf5_data);

  py::buffer_info field_buff_info = field.request();
  T* field_buff_data = static_cast<T*>(field_buff_info.ptr);

  for (std::size_t i = 0; i < dataset_dimensions[0]; i++)
  {
    for (std::size_t j = 0; j < dataset_dimensions[1]; j++)
    {
      for (std::size_t k = 0; k < dataset_dimensions[2]; k++)
      {
        field_buff_data[k + dataset_dimensions[2] * (j + dataset_dimensions[1] * i)] = hdf5_data[i][j][k];
      }
    }
  }

  return field;
}

} /* namespace seerep_hdf5_py */
