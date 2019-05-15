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

typedef uint32_t SceneId;
