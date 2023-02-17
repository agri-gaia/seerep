#ifndef CUSTOM_TIMER_H
#define CUSTOM_TIMER_H

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

/**
 * @brief RAII Timer to measure the time between it's creation and destruction.
 *
 * The start and end times as well as the duration will be append to a .csv file
 *
 */
class Timer
{
public:
  /**
   * @brief Construct the timer and start measuring the execution time
   *
   * @param fileName filename for the .csv file to output to. It get's created if not already present!
   */
  Timer(const std::string& fileName);
  ~Timer();

  void end();

private:
  const std::string& _fileName;
  std::chrono::time_point<std::chrono::high_resolution_clock> _start;
  std::chrono::time_point<std::chrono::high_resolution_clock> _end;
};
#endif  // CUSTOM_TIMER_H
