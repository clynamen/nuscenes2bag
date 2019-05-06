#pragma once

#include "nuscenes2bag/SampleQueue.hpp"
#include "nuscenes2bag/SampleSetWorker.hpp"
#include "nuscenes2bag/RadarObjects.h"


#include "rosbag/bag.h"

struct BagDescriptor {
  SceneId sceneId;
};

class MyProcessor : public SampleMsgProcessor {
public:
  MyProcessor(const std::string& bagName);

  void process(const TopicInfo &topicInfo,
                       SampleQueueConsumer<sensor_msgs::Image> &queueConsumer) override; 

  void process(const TopicInfo &topicInfo,
                       SampleQueueConsumer<sensor_msgs::PointCloud2> &queueConsumer) override; 

  void process(const TopicInfo &topicInfo,
                       SampleQueueConsumer<nuscenes2bag::RadarObjects> &queueConsumer) override; 

  rosbag::Bag* getBagForTopicInfo(const TopicInfo& topicInfo);

  virtual ~MyProcessor();

  std::vector<std::pair<BagDescriptor, std::unique_ptr<rosbag::Bag>>> bags;

  std::string templateBagName;
};