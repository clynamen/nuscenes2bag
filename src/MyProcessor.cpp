#include "nuscenes2bag/MyProcessor.hpp"

namespace fs = std::filesystem;

MyProcessor::MyProcessor(const std::string &bagName)
    : templateBagName(bagName) {
  // outBag.open(bagName, rosbag::bagmode::Write);
}

MyProcessor::~MyProcessor() {
  for (auto &bag : bags) {
    bag.second->close();
  }
}

rosbag::Bag *MyProcessor::getBagForTopicInfo(const TopicInfo &topicInfo) {
  for (auto &bag : bags) {
    if (bag.first.sceneId == topicInfo.sceneId) {
      return bag.second.get();
    }
  }
  bags.emplace_back(BagDescriptor{topicInfo.sceneId},
                    std::make_unique<rosbag::Bag>());

  std::string newBagFname = templateBagName + "_" + std::to_string(topicInfo.sceneId) +
                   ".bag";
  fs::remove(newBagFname);
  rosbag::Bag *newBag = bags.back().second.get();
  newBag->open(newBagFname,
               rosbag::bagmode::Write);
  return newBag;
}

void MyProcessor::process(
    const TopicInfo &topicInfo,
    SampleQueueConsumer<sensor_msgs::Image> &queueConsumer) {
  std::optional<sensor_msgs::Image> imageOpt = queueConsumer.get();

  if (imageOpt.has_value()) {
    sensor_msgs::Image image = imageOpt.value();

    // std::cout << "writing " << topicInfo.topicName << " at " <<
    // image.header.stamp << std::endl;
    auto outBag = getBagForTopicInfo(topicInfo);
    outBag->write(topicInfo.topicName, image.header.stamp, image);
  }
};

void MyProcessor::process(
    const TopicInfo &topicInfo,
    SampleQueueConsumer<sensor_msgs::PointCloud2> &queueConsumer) {
  std::optional<sensor_msgs::PointCloud2> imageOpt = queueConsumer.get();

  if (imageOpt.has_value()) {
    sensor_msgs::PointCloud2 image = imageOpt.value();

    auto outBag = getBagForTopicInfo(topicInfo);
    outBag->write(topicInfo.topicName, image.header.stamp, image);
  }
};

void MyProcessor::process(
    const TopicInfo &topicInfo,
    SampleQueueConsumer<nuscenes2bag::RadarObjects> &queueConsumer) {
  auto radarObjectsOpt = queueConsumer.get();

  if (radarObjectsOpt.has_value()) {
    auto radarObjects = radarObjectsOpt.value();

    auto outBag = getBagForTopicInfo(topicInfo);
    outBag->write(topicInfo.topicName, radarObjects.header.stamp, radarObjects);
  }
};