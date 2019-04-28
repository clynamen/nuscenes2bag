#include "nuscenes2rosbag/ImageDirectoryConverter.hpp"
#include "nuscenes2rosbag/utils.hpp"
#include <thread>

namespace fs = std::filesystem;

// ImageDirectoryConverter::ImageDirectoryConverter(
//     SampleQueueProducer<sensor_msgs::Image> &&queue,
//     const std::filesystem::path &path)
//     : SampleSetDirectoryConverter(path), queue(std::move(queue)) {}


// void ImageDirectoryConverter::processInternal() {
//   auto imageFiles = getFilesInSampleSetDirectoryOrderedByTime(directoryPath);

//   for (const auto &imageFile : imageFiles) {
//     if (!isRunning()) {
//       break;
//     }
//     cv::Mat image;
//     image = imread(imageFile.first.string().c_str(), cv::IMREAD_COLOR);
//     sensor_msgs::ImagePtr msg =
//         cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//     msg->header.stamp = stampUs2RosTime(imageFile.second.stampUs);
//     sensor_msgs::Image msgCopy = *msg;

//     while (true) {
//       if (queue.canPush()) {
//         queue.push(std::move(msgCopy));
//         break;
//       } else if (!isRunning()) {
//         break;
//       } else {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//       }
//     }
//   }

//   queue.close();
// }

template <>
sensor_msgs::Image processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo) {

  cv::Mat image;
  image = imread(fileInfo.first.string().c_str(), cv::IMREAD_COLOR);
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  msg->header.stamp = stampUs2RosTime(fileInfo.second.stampUs);
  return *msg;
}


template <>
class MsgDirectoryConverter<sensor_msgs::Image> : public SampleSetDirectoryConverter {};