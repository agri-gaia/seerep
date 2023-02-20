#ifndef MESSAGE_GENERATION_H
#define MESSAGE_GENERATION_H

#include <sensor_msgs/CompressedImage.h>

#include <algorithm>
#include <random>

#include "ros/ros.h"

using randomBytesEngine = std::independent_bits_engine<std::default_random_engine, CHAR_BIT, unsigned char>;

struct Config
{
  // size of one message in bytes
  long long int messagSize = 1024 * 1024;
  // totoal size to write in bytes
  long long int totalSize = 1024 * 1024 * 1024 * 10ULL;
};

sensor_msgs::CompressedImage generateMessage(Config config);

std::vector<sensor_msgs::CompressedImage> generateMessages(Config config);

#endif  // MESSAGE_GENERATION_H
