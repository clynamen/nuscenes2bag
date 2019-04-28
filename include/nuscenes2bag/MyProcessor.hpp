#pragma once

#include "nuscenes2bag/SampleQueue.hpp"
#include "nuscenes2bag/SampleSetWorker.hpp"

#include "rosbag/bag.h"

class MyProcessor : public SampleMsgProcessor {
public:
  MyProcessor(const std::string& bagName);

  virtual void process(const TopicInfo &topicInfo,
                       SampleQueueConsumer<sensor_msgs::Image> &queueConsumer) {
    std::optional<sensor_msgs::Image> imageOpt = queueConsumer.get();

    if(imageOpt.has_value()) {
      sensor_msgs::Image image = imageOpt.value();

      //std::cout << "writing " << topicInfo.topicName << " at " << image.header.stamp << std::endl;
      outBag.write(topicInfo.topicName, image.header.stamp, image);
    }

  };

  virtual ~MyProcessor();
  rosbag::Bag outBag;
};