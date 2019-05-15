#pragma once

#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>

std::optional<sensor_msgs::Image> readImageFile(std::filesystem::path filePath);
