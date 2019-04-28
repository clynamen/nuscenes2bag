#pragma once

#include "nuscenes2bag/SampleQueue.hpp"
#include "nuscenes2bag/SampleSetDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

template <class T>
T processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo);

template <>
sensor_msgs::Image processSingleFileFun(
    const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo);


template <class T>
class MsgDirectoryConverter : public SampleSetDirectoryConverter {
public:
  MsgDirectoryConverter(SampleQueueProducer<T> &&queue,
                        const std::filesystem::path &path) :  SampleSetDirectoryConverter(path), queue(queue) {}

  void processInternal() override {
    auto files = getFilesInSampleSetDirectoryOrderedByTime(directoryPath);

    for (const auto &file : files) {
      if (!isRunning()) {
        break;
      }

      T msg = processSingleFile(file);

      while (true) {
        if (queue.canPush()) {
          queue.push(std::move(msg));
          break;
        } else if (!isRunning()) {
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }

    queue.close();
  }

  T processSingleFile(
      const std::pair<std::filesystem::path, ExtractedFileNameInfo> &fileInfo) {
    return processSingleFileFun<T>(fileInfo);
  }

  SampleQueueProducer<T> queue;
};

// template <>
// class MsgDirectoryConverter<sensor_msgs::Image> : public
// SampleSetDirectoryConverter {

//   sensor_msgs::Image processSingleFile(const std::pair<std::filesystem::path,
//   ExtractedFileNameInfo>& fileInfo) {
//     cv::Mat image;
//     image = imread(fileInfo.first.string().c_str(), cv::IMREAD_COLOR);
//     sensor_msgs::ImagePtr msg =
//         cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//     msg->header.stamp = stampUs2RosTime(fileInfo.second.stampUs);
//     return *msg;
//   }
// };

// class ImageDirectoryConverter : public SampleSetDirectoryConverter {
// public:
//   ImageDirectoryConverter(SampleQueueProducer<sensor_msgs::Image> &&queue,
//                           const std::filesystem::path &path);

//   void processInternal() override;

//   SampleQueueProducer<sensor_msgs::Image> queue;
// };
