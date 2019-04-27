#include "nuscenes2rosbag/NuScenes2Rosbag.hpp"
#include "nuscenes2rosbag/SampleQueue.hpp"

#include <iostream>
#include <regex>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <thread>

namespace fs = std::filesystem;

NuScenes2Rosbag::NuScenes2Rosbag() {}

bool string_icontains(const std::string_view &string,
                      const std::string_view &sub) {
  std::string lowerString;
  std::string lowerSub;
  std::transform(string.begin(), string.end(), std::back_inserter(lowerString),
                 ::tolower);
  std::transform(sub.begin(), sub.end(), std::back_inserter(lowerSub),
                 ::tolower);
  return lowerString.find(lowerSub) != std::string::npos;
}

std::optional<FileSystemSampleSet>
NuScenes2Rosbag::extractSampleSetDescriptorInDirectory(
    const std::filesystem::path &inDirectoryPath) {
  const std::string &dirName = inDirectoryPath.filename();
  if (string_icontains(dirName, "CAM")) {
    return std::optional<FileSystemSampleSet>{
        {{dirName, SampleSetType::CAMERA}, inDirectoryPath}};
  } else if (string_icontains(dirName, "RADAR")) {
    return std::optional<FileSystemSampleSet>{
        {{dirName, SampleSetType::RADAR}, inDirectoryPath}};
  } else if (string_icontains(dirName, "LIDAR")) {
    return std::optional<FileSystemSampleSet>{
        {{dirName, SampleSetType::LIDAR}, inDirectoryPath}};
  } else {
    std::cout << "Skipping " << inDirectoryPath << std::endl;
  }
  return std::nullopt;
}

std::vector<FileSystemSampleSet> NuScenes2Rosbag::getSampleSetsInDirectory(
    const std::filesystem::path &inDatasetPath) {
  std::vector<FileSystemSampleSet> sets;

  for (auto &dir : fs::directory_iterator(inDatasetPath)) {
    if (dir.is_directory()) {
      auto sampleSetOpt = extractSampleSetDescriptorInDirectory(dir.path());
      if (sampleSetOpt.has_value()) {
        sets.push_back(sampleSetOpt.value());
      }
    }
  }

  return sets;
}

std::vector<FileSystemSampleSet> NuScenes2Rosbag::filterChosenSampleSets(
    const std::vector<FileSystemSampleSet> &sampleSets) {
  std::vector<FileSystemSampleSet> filteredSets;
  std::copy_if(sampleSets.begin(), sampleSets.end(),
               std::back_inserter(filteredSets), [](auto &fileSystemSampleSet) {
                 return fileSystemSampleSet.descriptor.setType >=
                        SampleSetType::CAMERA;
               });
  return filteredSets;
}

class MyProcessor : public SampleMsgProcessor {
public:
  virtual void process(const TopicInfo &topicInfo,
                       SampleQueueConsumer<sensor_msgs::Image> &queueConsumer) {
    std::optional<sensor_msgs::Image> imageOpt = queueConsumer.get();
    std::cout << "processing" << std::endl;
  };

  virtual ~MyProcessor() = default;
};

class SampleSetDirectoryConverter {
public:
  SampleSetDirectoryConverter(const std::filesystem::path &path);
  virtual ~SampleSetDirectoryConverter();

  void process();
  virtual void processInternal() = 0;

  void isRunning();
  void stop();

protected:
  std::filesystem::path directoryPath;

private:
  bool running;
};

class ImageDirectoryConverter : public SampleSetDirectoryConverter {
  void processInternal() override;
};

struct ExtractedFileNameInfoImage{
  int sceneId;
  uint64_t stampUs;

};

static std::regex IMAGE_REGEX("^n(\\d+)-.*__(\\d+)\\.jpg");

std::optional<ExtractedFileNameInfoImage> getInfoFromFilename(const std::string& fname) {
  int sceneId;
  uint64_t stampUs;

  std::smatch m;
  auto matched = std::regex_search(fname, m, IMAGE_REGEX);
  if(matched) {
    sceneId = std::stoi(m[0]);
    stampUs = std::stoull(m[1]);
    return std::optional<ExtractedFileNameInfoImage>{{sceneId, stampUs}};
  }

  return std::nullopt;
}

void ImageDirectoryConverter::processInternal() {
  for (const auto &entry : fs::directory_iterator(directoryPath)) {
    if(entry.is_regular_file()) {
      auto fileInfo = getInfoFromFilename(entry.path().filename());
    }
  }
}

SampleSetDirectoryConverter::SampleSetDirectoryConverter(
    const std::filesystem::path &path)
    : directoryPath(path), running(false) {}

SampleSetDirectoryConverter::~SampleSetDirectoryConverter() {}

void SampleSetDirectoryConverter::process() {
  running = true;
  processInternal();
}

void SampleSetDirectoryConverter::stop() { running = false; }

void NuScenes2Rosbag::convertDirectory(
    const std::filesystem::path &inDatasetPath,
    const std::filesystem::path &outputRosbagPath) {
  //   rosbag::Bag outBag;
  //   outBag.open(outputRosbagPath.string(), rosbag::bagmode::Write);

  auto availableSampleSets = getSampleSetsInDirectory(inDatasetPath);
  std::cout << "Found " << availableSampleSets.size()
            << " valid sample directory" << std::endl;

  auto chosenSets = filterChosenSampleSets(availableSampleSets);
  std::cout << "Chosen " << chosenSets.size() << " sample directory"
            << std::endl;

  // instantiate produces
  std::vector<std::pair<TopicInfo, TypeErasedQueue>> typeErasedQueueList;

  auto [producer, consumer] =
      SampleQueueFactory<sensor_msgs::Image>::makeQueue();
  typeErasedQueueList.emplace_back(TopicInfo(std::string("asdf")),
                                   TypeErasedQueue(consumer));

  std::thread t([prod = std::ref(producer)]() {
    sensor_msgs::Image m1;
    sensor_msgs::Image m2;
    prod.get().push(std::move(m1));
    prod.get().push(std::move(m2));
    prod.get().close();
  });
  //   typeErasedQueueList.push_back(TypeErasedQueue(a));
  //   typeErasedQueueList.push_back(TypeErasedQueue(b));

  MyProcessor processor;

  while (true) {
    bool atLeastOneQueueIsStillOpen = false;
    for (auto &[topicInfo, queue] : typeErasedQueueList) {
      if (!queue.isClosed() || (queue.size() > 0)) {
        //   if (!queue.isClosed() ) {
        atLeastOneQueueIsStillOpen = true;
        queue.process(topicInfo, processor);
      }
    }
    if (!atLeastOneQueueIsStillOpen) {
      std::cout << "completed all queue" << std::endl;
      break;
    }
  }

  t.join();

  // std_msgs::String str;
  // str.data = std::string("foo");

  // std_msgs::Int32 i;
  // i.data = 42;

  // bag.write("chatter", ros::Time::now(), str);
  // bag.write("numbers", ros::Time::now(), i);

  //   outBag.close();
}