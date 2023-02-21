#ifndef ANALYSIS_H
#define ANALYSIS_H

#define MCAP_IMPLEMENTATION  // Define this in exactly one .cpp file

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <chrono>
#include <mcap/writer.hpp>
#include <string>

#include "message-generation.h"
#include "timer.h"
#include "seerep-hdf5-ros/hdf5-ros.h"

constexpr std::string_view TOPIC = "/camera_c/depth/image_rect_raw";
constexpr std::string_view MCAP_FILE_PATH = "/home/pbrstudent/Documents/seerep-data/output.mcap";
const std::string HDF5_FILE_PATH = "/home/pbrstudent/Documents/seerep-data/output.hdf5";

mcap::Timestamp now();

template <typename T>
void saveInMCAP(const std::vector<T>& messages);

template <typename T>
void saveInHdf5(const std::vector<T>& messages);

#endif  // MCAP_TEST_H