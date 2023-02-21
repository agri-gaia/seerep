#include "message-generation.h"

/**
 * The loadData(...) and the getMessageData(...) functions are taken from the rosbag2_performance package:
 * https://github.com/james-rms/rosbag2/blob/plugin-comparison/rosbag2_performance/rosbag2_storage_plugin_comparison/src/single_benchmark.cpp
 */

std::vector<unsigned char> loadData(const char* filePath)
{
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
    if (end > data.size())
    {
      end = data.size();
    }
    output.insert(output.end(), &data[start], &data[end]);
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

sensor_msgs::CompressedImage generateMessage(const std::vector<unsigned char>& data)
{
  sensor_msgs::CompressedImage message;

  message.header.frame_id = "map";
  message.header.stamp = ros::Time::now();
  message.format = "jpeg";
  message.data.assign(data.begin(), data.end());

  return message;
}

std::vector<sensor_msgs::CompressedImage> generateMessages(const Config& config, const std::vector<unsigned char>& data)
{
  size_t numMessages = std::ceil(config.totalSize / (config.messageSize * 1.0));
  std::cout << "Number of messages to be written: " << numMessages << std::endl;

  std::vector<sensor_msgs::CompressedImage> messages;
  messages.reserve(numMessages);

  size_t offset = 0;
  for (size_t i = 0; i < numMessages; i++)
  {
    auto [messageData, newOffset] = getMessageData(offset, config.messageSize, data);
    messages.insert(messages.end(), generateMessage(messageData));
  }

  // TODO shuffle the messages

  return messages;
}
