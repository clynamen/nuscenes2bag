#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <thread>

namespace fs = std::filesystem;

std::optional<sensor_msgs::Image> readImageFile(std::filesystem::path filePath) {
  cv::Mat image;
  image = imread(filePath.string().c_str(), cv::IMREAD_COLOR);
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  return std::optional(*msg);
}

template <>
std::optional<sensor_msgs::Image> processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo) {

  auto msgOpt = readImageFile(fileInfo.first);
  if(!msgOpt.has_value()) {
    return std::nullopt;
  }
  auto msg = *msgOpt;
  msg.header.stamp = stampUs2RosTime(fileInfo.second.stampUs);
  return msg;
}


template <>
class MsgDirectoryConverter<sensor_msgs::Image> : public SampleSetDirectoryConverter {};