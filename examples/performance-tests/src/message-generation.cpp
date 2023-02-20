#include "message-generation.h"

sensor_msgs::CompressedImage generateMessage(Config config)
{
  sensor_msgs::CompressedImage message;
  ros::Time::init();

  randomBytesEngine rbe;
  std::vector<unsigned char> data(config.messagSize);
  std::generate(begin(data), end(data), std::ref(rbe));

  message.header.frame_id = "map";
  message.header.stamp = ros::Time::now();
  message.format = "jpeg";
  message.data = std::move(data);

  return message;
}

std::vector<sensor_msgs::CompressedImage> generateMessages(Config config)
{
  size_t numMessages = std::ceil(config.totalSize / (config.messagSize * 1.0));
  std::vector<sensor_msgs::CompressedImage> messages;
  messages.reserve(numMessages);
  for (size_t i = 0; i < numMessages; i++)
  {
    messages.push_back(generateMessage(config));
  }
  return messages;
}
