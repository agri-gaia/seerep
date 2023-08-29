#ifndef ANALYSIS_H
#define ANALYSIS_H

#define MCAP_IMPLEMENTATION  // Define this in exactly one .cpp file

#include <cstdlib>
#include <mcap/types.hpp>
#include <mcap/writer.hpp>
#include <string>

#include "message-generation.h"
#include "seerep-hdf5-ros/hdf5-ros.h"

using hrc = std::chrono::high_resolution_clock;

mcap::Timestamp now();

std::pair<std::chrono::nanoseconds, double> summerize_run(const std::vector<std::chrono::nanoseconds>& durations,
                                                          std::vector<double>& written_bytes);

void save_run(std::pair<std::chrono::nanoseconds, double> summerized_run, const std::string& csv_path);

template <typename T>
void saveInMCAP(const std::vector<T>& messages, const std::string& outputDir, const std::string& label);

template <typename T>
void saveInHdf5(const std::vector<T>& messages, const std::string& outputDir, const std::string& label);

#endif  // ANALYSIS_H
