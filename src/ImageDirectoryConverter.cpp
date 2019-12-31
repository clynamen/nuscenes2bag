#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <thread>

namespace nuscenes2bag {

boost::optional<sensor_msgs::Image>
readImageFile(const fs::path& filePath) noexcept
{
  cv::Mat image;
  try {
    image = imread(filePath.string().c_str(), cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    return boost::optional<sensor_msgs::Image>(*msg);

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);
  }

  return boost::none;
}

}