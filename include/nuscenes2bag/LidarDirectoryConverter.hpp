#pragma once

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <filesystem>

namespace nuscenes2bag {

std::optional<sensor_msgs::PointCloud2> readLidarFile(const std::filesystem::path& filePath);

}
