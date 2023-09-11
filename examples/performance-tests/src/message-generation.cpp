#include "message-generation.h"

/**
 * The loadData(...) and the getMessageData(...) functions are taken from the rosbag2_performance package:
 * https://github.com/james-rms/rosbag2/blob/plugin-comparison/rosbag2_performance/rosbag2_storage_plugin_comparison/src/single_benchmark.cpp
 */

std::vector<unsigned char> loadData(const char* filePath)
{
  if (!std::filesystem::exists(filePath))
  {
    throw std::runtime_error("File does not exist");
  }
  std::ifstream file(filePath, std::ios::binary);
  return std::vector<unsigned char>((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

std::pair<std::vector<unsigned char>, size_t> getMessageData(size_t start, size_t size,
                                                             const std::vector<unsigned char>& data)
{
  std::vector<unsigned char> output;
  output.reserve(size);

  while (output.size() < size)
  {
    size_t end = start + size;

    // make shure we don't read more than we have
    if (end > data.size())
    {
      end = data.size();
    }

    // make shure we don't write more than we need
    if (output.size() + end - start > size)
    {
      end = start + size - output.size();
    }

    output.insert(output.end(), &data[start], &data[end]);

    // check if we need to wrap around
    if (end != data.size())
    {
      start = end;
    }
    else
    {
      start = 0;
    }
  }

  return std::make_pair(output, start);
}

// TODO: this could be replaced with a cutom message that only contains a data field
sensor_msgs::CompressedImage generateMessage(const std::vector<unsigned char>& data)
{
  sensor_msgs::CompressedImage message;

  message.header.frame_id = "map";
  message.header.stamp = ros::Time::now();
  message.format = "jpeg";
  message.data = std::move(data);

  return message;
}

std::vector<sensor_msgs::CompressedImage> generateMessages(const Config& config, const std::vector<unsigned char>& data)
{
  size_t written_bytes = 0, write_size = 0, offset = 0;

  std::vector<sensor_msgs::CompressedImage> messages;
  messages.reserve(std::ceil(config.totalSize / config.messageSize));

  while (written_bytes < config.totalSize)
  {
    write_size =
        written_bytes + config.messageSize > config.totalSize ? config.totalSize - written_bytes : config.messageSize;

    auto [message_data, new_offset] = getMessageData(offset, write_size, data);
    messages.insert(messages.end(), generateMessage(message_data));

    offset = new_offset;
    written_bytes += write_size;
  }

  auto rng = std::default_random_engine{};
  std::shuffle(std::begin(messages), std::end(messages), rng);

  return messages;
}
