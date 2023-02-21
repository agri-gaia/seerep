#include "message-generation.h"

sensor_msgs::CompressedImage generateMessage(Config config)
{
  sensor_msgs::CompressedImage message;
  ros::Time::init();

  std::vector<unsigned char> data;
  data.resize(config.messageSize);

  int fd = open("/dev/random", O_RDONLY);
  if (fd < 0)
  {
    fprintf(stderr, "Error while opening the file\n");
  }

  int ret = read(fd, &data[0], config.messageSize);

  if (ret < 0)
  {
    fprintf(stderr, "Error: %s\n", strerror(errno));
  }
  close(fd);

  message.header.frame_id = "map";
  message.header.stamp = ros::Time::now();
  message.format = "jpeg";
  message.data.assign(data.begin(), data.end());

  return message;
}

std::vector<sensor_msgs::CompressedImage> generateMessages(Config config)
{
  size_t numMessages = std::ceil(config.totalSize / (config.messageSize * 1.0));
  std::vector<sensor_msgs::CompressedImage> messages;
  messages.reserve(numMessages);
  for (size_t i = 0; i < numMessages; i++)
  {
    messages.push_back(generateMessage(config));
  }

  std::cout << "Number of messages: " << messages.size() << std::endl;

  return messages;
}
