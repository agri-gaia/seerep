#ifndef MESSAGE_GENERATION_H
#define MESSAGE_GENERATION_H

#include <fcntl.h>
#include <sensor_msgs/CompressedImage.h>
#include <unistd.h>

#include <algorithm>

#include "ros/ros.h"

struct Config
{
  // size of one message in bytes
  long long int messageSize = 1024 * 1024;
  // totoal size to write in bytes
  long long int totalSize = 1024 * 1024 * 1024 * 15ULL;
};

sensor_msgs::CompressedImage generateMessage(Config config);

std::vector<sensor_msgs::CompressedImage> generateMessages(Config config);

#endif  // MESSAGE_GENERATION_H
