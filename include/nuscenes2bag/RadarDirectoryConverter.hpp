
#pragma once

#include "nuscenes2bag/MsgDirectoryConverter.hpp"
#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/PclRadarObject.hpp"

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>


std::optional<nuscenes2bag::RadarObjects> readRadarFile(const std::filesystem::path& filePath);

template <>
std::optional<nuscenes2bag::RadarObjects> processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo);