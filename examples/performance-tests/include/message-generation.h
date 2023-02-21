#ifndef MESSAGE_GENERATION_H
#define MESSAGE_GENERATION_H

#include <sensor_msgs/CompressedImage.h>

#include <algorithm>
#include <fstream>
#include <random>

#include "ros/ros.h"

struct Config
{
  // size of one message in bytes
  long long int messageSize = 1024 * 1024;
  // totoal size to write in bytes
  long long int totalSize = 1024 * 1024 * 1024 * 15ULL;
};

std::vector<unsigned char> loadData(const char* filePath);

std::pair<std::vector<unsigned char>, size_t> getMessageData(size_t start, size_t size,
                                                             const std::vector<unsigned char>& data);

sensor_msgs::CompressedImage generateMessage(const std::vector<unsigned char>& data);

std::vector<sensor_msgs::CompressedImage> generateMessages(const Config& config, const std::vector<unsigned char>& data);

#endif  // MESSAGE_GENERATION_H
