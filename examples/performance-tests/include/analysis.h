#ifndef ANALYSIS_H
#define ANALYSIS_H

#define MCAP_IMPLEMENTATION  // Define this in exactly one .cpp file

#include <cstdlib>
#include <mcap/types.hpp>
#include <mcap/writer.hpp>
#include <string>

#include "message-generation.h"
#include "seerep-hdf5-ros/hdf5-ros.h"
#include "timer.h"

mcap::Timestamp now();

template <typename T>
void saveInMCAP(const std::vector<T>& messages, const std::string& outputDir, const std::string& label);

template <typename T>
void saveInHdf5(const std::vector<T>& messages, const std::string& outputDir, const std::string& label);

#endif  // ANALYSIS_H
