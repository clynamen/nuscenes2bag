#pragma once

#include <array>
#include <iostream>
#include <sstream>
#include <string>

#include "nuscenes2bag/ToDebugString.hpp"

namespace nuscenes2bag {

enum class SampleType
{
  NONE,
  CAMERA,
  RADAR,
  LIDAR
};

typedef std::string Token;
typedef uint64_t TimeStamp;
typedef uint32_t SceneId;
typedef std::array<std::array<double, 3>, 3> IntrinsicsMatrix;

}
