#pragma once

#include <chrono>

namespace loam
{ 
  /** \brief A standard non-ROS alternative to ros::Time.*/
  //注：使用using设别名和typedef效果相同
  using Time = std::chrono::system_clock::time_point;

  // helper function
  inline double toSec(Time::duration duration)
  {
    return std::chrono::duration<double>(duration).count();
  };
}
