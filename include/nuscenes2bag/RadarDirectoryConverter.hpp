#pragma once

#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/PclRadarObject.hpp"

#include <filesystem>


std::optional<nuscenes2bag::RadarObjects> readRadarFile(const std::filesystem::path& filePath);
