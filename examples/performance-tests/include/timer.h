#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

/* Simple RAII timer to record it's live time and output the data into a .cvs file*/
class Timer
{
public:
  Timer(const std::string& fileName);
  ~Timer();

  void end();

private:
  const std::string& _fileName;
  std::chrono::time_point<std::chrono::high_resolution_clock> _start;
  std::chrono::time_point<std::chrono::high_resolution_clock> _end;
};
#endif  // TIMER_H
