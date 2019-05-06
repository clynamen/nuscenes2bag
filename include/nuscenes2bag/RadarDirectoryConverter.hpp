
#pragma once

#include "nuscenes2bag/MsgDirectoryConverter.hpp"
#include "nuscenes2bag/RadarObjects.h"

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

template <>
std::optional<nuscenes2bag::RadarObjects> processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo);