#pragma once

#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/PclRadarObject.hpp"

#include <filesystem>

namespace nuscenes2bag {

std::optional<nuscenes2bag::RadarObjects> readRadarFile(const std::filesystem::path& filePath);

}