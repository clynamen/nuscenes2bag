#pragma once

#include "nuscenes2rosbag/SampleQueue.hpp"
#include "nuscenes2rosbag/SampleSetDirectoryConverter.hpp"
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <regex>

class ImageDirectoryConverter : public SampleSetDirectoryConverter {
public:
  ImageDirectoryConverter(SampleQueueProducer<sensor_msgs::Image> &&queue,
                          const std::filesystem::path &path);

  void processInternal() override;

  SampleQueueProducer<sensor_msgs::Image> queue;
};

struct ExtractedFileNameInfoImage {
  int sceneId;
  uint64_t stampUs;
};

static std::regex IMAGE_REGEX("^n(\\d+)-.*__(\\d+)\\.jpg");

std::optional<ExtractedFileNameInfoImage>
getInfoFromFilename(const std::string &fname);