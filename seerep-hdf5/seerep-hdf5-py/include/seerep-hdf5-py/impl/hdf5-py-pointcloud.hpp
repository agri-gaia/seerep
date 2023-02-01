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
                                      std::map<std::string, bool>& processed,
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

  // mark channels as processed
  for (const auto& channel_name : channel_names)
  {
    processed[channel_name] = true;
  }

  return true;
}

} /* namespace seerep_hdf5_py */
