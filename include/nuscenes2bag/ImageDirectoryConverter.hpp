#pragma once

#include "nuscenes2bag/MsgDirectoryConverter.hpp"

#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

template <>
std::optional<sensor_msgs::Image> processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo);