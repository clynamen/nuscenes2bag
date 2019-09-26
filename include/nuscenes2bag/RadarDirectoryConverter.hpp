#pragma once

#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/PclRadarObject.hpp"

#if CMAKE_CXX_STANDARD >= 17
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

namespace nuscenes2bag {

#if CMAKE_CXX_STANDARD >= 17
std::optional<nuscenes2bag::RadarObjects> readRadarFile(const fs::path& filePath);
#else
nuscenes2bag::RadarObjectsPtr readRadarFile(const fs::path& filePath);
#endif

}