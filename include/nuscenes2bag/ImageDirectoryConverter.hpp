#pragma once

#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#if CMAKE_CXX_STANDARD >= 17
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

namespace nuscenes2bag {

#if CMAKE_CXX_STANDARD >= 17
std::optional<sensor_msgs::Image> readImageFile(const fs::path& filePath) noexcept;
#else
sensor_msgs::ImagePtr readImageFile(const fs::path& filePath) noexcept;
#endif

}
