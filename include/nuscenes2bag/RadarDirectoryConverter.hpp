#pragma once

#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/PclRadarObject.hpp"
#include "nuscenes2bag/Filesystem.hpp"

namespace nuscenes2bag {

boost::optional<nuscenes2bag::RadarObjects> readRadarFile(const fs::path& filePath);

}