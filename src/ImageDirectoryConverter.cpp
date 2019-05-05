#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <thread>

namespace fs = std::filesystem;


template <>
std::optional<sensor_msgs::Image> processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo) {

  cv::Mat image;
  image = imread(fileInfo.first.string().c_str(), cv::IMREAD_COLOR);
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  msg->header.stamp = stampUs2RosTime(fileInfo.second.stampUs);
  return std::optional(*msg);
}


template <>
class MsgDirectoryConverter<sensor_msgs::Image> : public SampleSetDirectoryConverter {};