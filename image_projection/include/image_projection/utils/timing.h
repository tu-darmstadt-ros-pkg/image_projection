#ifndef TIMING_H
#define TIMING_H

#include <ros/ros.h>

#include <iostream>
#include <sstream>
#include <stack>
#include <unordered_map>
#include <chrono>

namespace timing {

  inline std::string line(unsigned int length) {
    std::string l = "";
    for (unsigned int i = 0; i < length; i++) {
      l += "-";
    }
    return l;
  }

// Allocates memory for static variables, use in .cpp
  #define INIT_TIMING \
  int timing::IDFactory::id = 0; \
  std::unordered_map<int, timing::TimingInfo> timing::Timing::timing_infos_; \
  std::stack<timing::TimePoint> timing::Timing::timing_stack_; \
  bool timing::Timing::dont_print_times_ = false; \
  unsigned int timing::Timing::start_counter_ = 0; \
  unsigned int timing::Timing::stop_counter_ = 0; \

  #define START_TIMING(timerName) \
  timing::Timing::startTiming(timerName);

  #define STOP_TIMING \
  timing::Timing::stopTiming(false);

  #define STOP_TIMING_PRINT \
  timing::Timing::stopTiming(true);

  #define STOP_TIMING_AVG \
  { \
  static int timer_id = -1; \
  timing::Timing::stopTiming(false, timer_id); \
  }

  #define STOP_TIMING_AVG_PRINT \
  { \
  static int timer_id = -1; \
  timing::Timing::stopTiming(true, timer_id); \
  }

  /** \brief Struct to store a time measurement.
  */
  struct TimePoint
  {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::string name;
  };

  /** \brief Struct to store the total time and the number of steps in order to compute the average time.
  */
  struct TimingInfo
  {
    double max;
    double total_time;
    unsigned int counter;
    std::string name;
  };

  /** \brief Factory for unique ids.
  */
  class IDFactory
  {
  private:
    /** Current id */
    static int id;

  public:
    static void resetId() {id = 0;}
    static int getId() { return id++; }
  };

  /** \brief Class for time measurements.
  */
  class Timing
  {
  public:
    static bool dont_print_times_;
    static unsigned int start_counter_;
    static unsigned int stop_counter_;
    static std::stack<TimePoint> timing_stack_;
    static std::unordered_map<int, TimingInfo> timing_infos_;

    static void reset()
    {
      while (!timing_stack_.empty())
        timing_stack_.pop();
      timing_infos_.clear();
      start_counter_ = 0;
      stop_counter_ = 0;
    }

    static void startTiming(const std::string& name = std::string(""))
    {
      TimePoint tp;
      tp.start = std::chrono::high_resolution_clock::now();
      tp.name = name;
      Timing::timing_stack_.push(tp);
      Timing::start_counter_++;
    }

    static double stopTiming(bool print = true)
    {
      if (!Timing::timing_stack_.empty())
      {
        Timing::stop_counter_++;
        std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
        TimePoint tp = Timing::timing_stack_.top();
        Timing::timing_stack_.pop();
        std::chrono::duration<double> elapsed_seconds = stop - tp.start;
        double t = elapsed_seconds.count() * 1000.0;

        if (print)
          ROS_INFO_STREAM("Time '" << tp.name.c_str() << "': " << t << " ms");
        return t;
      }
      return 0;
    }

    static double stopTiming(bool print, int &id)
    {
      if (id == -1)
        id = IDFactory::getId();
      if (!Timing::timing_stack_.empty())
      {
        Timing::stop_counter_++;
        std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
        TimePoint tp = Timing::timing_stack_.top();
        Timing::timing_stack_.pop();

        std::chrono::duration<double> elapsed_seconds = stop - tp.start;
        double t = elapsed_seconds.count() * 1000.0;

        if (print && !Timing::dont_print_times_)
          ROS_INFO_STREAM("Time " << tp.name.c_str() << ": " << t << " ms");

        if (id >= 0)
        {
          std::unordered_map<int, TimingInfo>::iterator iter;
          iter = Timing::timing_infos_.find(id);
          if (iter != Timing::timing_infos_.end())
          {
            Timing::timing_infos_[id].total_time += t;
            Timing::timing_infos_[id].counter++;
            if (t > Timing::timing_infos_[id].max) {
              Timing::timing_infos_[id].max = t;
            }
          }
          else
          {
            TimingInfo ti;
            ti.counter = 1;
            ti.max = t;
            ti.total_time = t;
            ti.name = tp.name;
            Timing::timing_infos_[id] = ti;
          }
        }
        return t;
      }
      return 0;
    }



    static void printTimeInfos()
    {
      // Find longest name
      unsigned int column_width_name = 0;
      for (auto it = begin(Timing::timing_infos_); it != end(Timing::timing_infos_); ++it) {
        const TimingInfo &info = it->second;
        if (info.name.size() > column_width_name) {
          column_width_name = info.name.size();
        }
      }
      const int column_width_avg = 10;
      const int column_width_max = 10;
      const int column_width_sum = 10;
      const int column_width_count = 10;
      std::stringstream ss;
      ss << "TIMING INFOS:" << std::endl;
      ss << "|" << std::setw(column_width_name) << "Name " << "|" << std::setw(column_width_avg) << "Avg (ms)" << "|" << std::setw(column_width_max) << "Max (ms)" << "|"
         << std::setw(column_width_sum) << "Sum (ms)" << "|" << std::setw(column_width_count) << "Count" << "|" << std::endl;
      ss << line(column_width_name + column_width_avg + column_width_max + column_width_sum + column_width_count + 5) << std::endl;

      for (auto it = begin(Timing::timing_infos_); it != end(Timing::timing_infos_); ++it) {
        const TimingInfo &info = it->second;
        double avg = info.total_time / info.counter;
        ss << "|" << std::setw(column_width_name) << info.name << "|" << std::setw(column_width_avg) << avg << "|" << std::setw(column_width_max) << info.max << "|"
           << std::setw(column_width_sum) << info.total_time << "|" << std::setw(column_width_count) << info.counter << "|" << std::endl;
      }
      std::cout << ss.str();
    }

    static void printAverageTimes()
    {   
      std::unordered_map<int, TimingInfo>::iterator iter;
      for (iter = Timing::timing_infos_.begin(); iter != Timing::timing_infos_.end(); iter++)
      {
        const TimingInfo &at = iter->second;
        double avgTime = at.total_time / at.counter;
        ROS_INFO_STREAM("Average time '" << at.name.c_str() << "': " << avgTime << " ms [" << at.counter << "]");
      }
      if (Timing::start_counter_ != Timing::stop_counter_)
        ROS_INFO_STREAM("Problem: " << Timing::start_counter_ << " calls of startTiming and " << Timing::stop_counter_ << " calls of stopTiming.");
    }

    static void printTimeSums()
    {
      std::unordered_map<int, TimingInfo>::iterator iter;
      for (iter = Timing::timing_infos_.begin(); iter != Timing::timing_infos_.end(); iter++)
      {
        const TimingInfo &at = iter->second;
        const double timeSum = at.total_time;
        ROS_INFO_STREAM("Time sum " << at.name.c_str() << ": " << timeSum << " ms");
      }
      if (Timing::start_counter_ != Timing::stop_counter_)
        ROS_INFO_STREAM("Problem: " << Timing::start_counter_ << " calls of startTiming and " << Timing::stop_counter_ << " calls of stopTiming. ");
    }
  };
}

#endif
