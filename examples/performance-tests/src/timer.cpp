#include "timer.h"

Timer::Timer(const std::string& fileName) : _fileName(fileName)
{
  _start = std::chrono::high_resolution_clock::now();
}

Timer::~Timer()
{
  end();
}

void Timer::end()
{
  _end = std::chrono::high_resolution_clock::now();

  auto startNum = std::chrono::time_point_cast<std::chrono::milliseconds>(_start).time_since_epoch().count();
  auto endNum = std::chrono::time_point_cast<std::chrono::milliseconds>(_end).time_since_epoch().count();

  auto duration = endNum - startNum;

  std::stringstream ss;
  ss << startNum << "," << endNum << "," << duration << ",\n";

  std::ofstream file;
  file.open(_fileName, std::ios::app);
  file << ss.str();
  file.close();
}
