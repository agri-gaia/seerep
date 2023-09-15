#ifndef MESSAGE_GENERATION_H
#define MESSAGE_GENERATION_H

#include <sensor_msgs/CompressedImage.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>

#include "ros/ros.h"

struct Config
{
  // size of one message in bytes
  size_t messageSize = 0;
  // totoal size to write in bytes
  size_t totalSize = 0;
};

std::vector<unsigned char> loadData(const char* filePath);

std::pair<std::vector<unsigned char>, size_t> getMessageData(size_t start, size_t size,
                                                             const std::vector<unsigned char>& data);

sensor_msgs::CompressedImage generateMessage(const std::vector<unsigned char>& data);

std::vector<sensor_msgs::CompressedImage> generateMessages(const Config& config, const std::vector<unsigned char>& data);

#endif  // MESSAGE_GENERATION_H
