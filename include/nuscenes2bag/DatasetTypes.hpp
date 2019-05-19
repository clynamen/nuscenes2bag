#pragma once

#include <filesystem>
#include <sstream>
#include <string>
#include <iostream>

#include "nuscenes2bag/ToDebugString.hpp"

enum class SampleType {
    CAMERA,
    RADAR,
    LIDAR
};

typedef std::string Token;
typedef uint64_t TimeStamp;
typedef uint32_t SceneId;
typedef std::array<std::array<double, 3>, 3> IntrinsicsMatrix;