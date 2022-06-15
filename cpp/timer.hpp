#pragma once
#include <chrono>
#include <cstdlib>
#include <string>

class Timer {
  typedef std::chrono::high_resolution_clock Time;
  std::string last_title_;
  decltype(Time::now()) last_start_, last_end_;

 public:
  void start(std::string const& title) {
    last_title_ = title;
    last_start_ = Time::now();
  }
  void stop(bool verbose = true) {
    last_end_ = Time::now();
    if (verbose) print();
  }
  void print(FILE* fp = stderr) const {
    std::chrono::duration<float> elapsed = last_end_ - last_start_;
    fprintf(fp, "%s: %.2f\n", last_title_.c_str(), elapsed.count());
  }
};
