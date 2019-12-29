#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <thread>

namespace nuscenes2bag {

#if CMAKE_CXX_STANDARD >= 17
std::optional<sensor_msgs::Image> readImageFile(const fs::path& filePath) noexcept
#else
sensor_msgs::ImagePtr readImageFile(const fs::path& filePath) noexcept
#endif
{
  cv::Mat image;
  try {
    image = imread(filePath.string().c_str(), cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

#if CMAKE_CXX_STANDARD >= 17
    return std::optional(*msg);
#else
    return msg;
#endif

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);
  }

#if CMAKE_CXX_STANDARD >= 17
  return std::nullopt;
#else
  sensor_msgs::ImagePtr empty_msg;
  return empty_msg;
#endif

}

}