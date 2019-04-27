#include "nuscenes2rosbag/NuScenes2Rosbag.hpp"
#include "nuscenes2rosbag/SampleQueue.hpp"

#include <boost/asio.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

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
  MyProcessor(const std::string& bagName);

  virtual void process(const TopicInfo &topicInfo,
                       SampleQueueConsumer<sensor_msgs::Image> &queueConsumer) {
    std::optional<sensor_msgs::Image> imageOpt = queueConsumer.get();

    if(imageOpt.has_value()) {
      sensor_msgs::Image image = imageOpt.value();
      std::cout << "writing " << topicInfo.topicName << " at " << image.header.stamp << std::endl;
      outBag.write(topicInfo.topicName, image.header.stamp, image);
    }

  };

  virtual ~MyProcessor();
  rosbag::Bag outBag;
};

MyProcessor::MyProcessor(const std::string& bagName) {
  outBag.open(bagName, rosbag::bagmode::Write);
}

MyProcessor::~MyProcessor() {
  outBag.close();
}

class SampleSetDirectoryConverter {
public:
  SampleSetDirectoryConverter(const std::filesystem::path &path);
  virtual ~SampleSetDirectoryConverter();

  void process();

  bool isRunning();
  void stop();

protected:
  virtual void processInternal() = 0;

  std::filesystem::path directoryPath;

private:
  bool running;
};

class ImageDirectoryConverter : public SampleSetDirectoryConverter {
public:
  ImageDirectoryConverter(SampleQueueProducer<sensor_msgs::Image> &&queue,
                          const std::filesystem::path &path);

  void processInternal() override;

  SampleQueueProducer<sensor_msgs::Image> queue;
};

ImageDirectoryConverter::ImageDirectoryConverter(
    SampleQueueProducer<sensor_msgs::Image> &&queue,
    const std::filesystem::path &path)
    : SampleSetDirectoryConverter(path), queue(std::move(queue)) {}

struct ExtractedFileNameInfoImage {
  int sceneId;
  uint64_t stampUs;
};

static std::regex IMAGE_REGEX("^n(\\d+)-.*__(\\d+)\\.jpg");

std::optional<ExtractedFileNameInfoImage>
getInfoFromFilename(const std::string &fname) {
  int sceneId;
  uint64_t stampUs;

  std::smatch m;
  auto matched = std::regex_search(fname, m, IMAGE_REGEX);
  if (matched) {
    try {
      sceneId = std::stoi(m[1]);
      stampUs = std::stoull(m[2]);
      return std::optional<ExtractedFileNameInfoImage>{{sceneId, stampUs}};
    } catch (const std::exception &e) {
      std::cout << "Unable to parse " << fname << std::endl;
    }
  }

  return std::nullopt;
}

ros::Time stampUs2RosTime(uint64_t stampUs) {
  ros::Time t;
  t = t.fromNSec(stampUs * 1000);
  return t;
}

void ImageDirectoryConverter::processInternal() {
  std::vector<std::pair<fs::path, ExtractedFileNameInfoImage>> imageFiles;

  for (const auto &entry : fs::directory_iterator(directoryPath)) {
    if (!isRunning()) {
      break;
    }
    if (entry.is_regular_file()) {
      const auto &fname = entry.path().filename();
      auto fileInfoOpt = getInfoFromFilename(fname);
      if (fileInfoOpt.has_value()) {
        auto fileInfo = fileInfoOpt.value();
        // std::cout << "Processing image " << fileInfo.sceneId << "  " <<
        // fileInfo.stampUs << std::endl;
        imageFiles.emplace_back(entry.path(), fileInfo);
      } else {
        std::cout << "Skipping " << fname << std::endl;
      }
    }
  }

  std::sort(imageFiles.begin(), imageFiles.end(),
            [](const auto &l, const auto &r) {
              return l.second.stampUs < r.second.stampUs;
            });

  for (const auto &imageFile : imageFiles) {
    if (!isRunning()) {
      break;
    }
    cv::Mat image;
    image = imread(imageFile.first.string().c_str(), cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = stampUs2RosTime(imageFile.second.stampUs);
    sensor_msgs::Image msgCopy = *msg;

    while (true) {
      if (queue.canPush()) {
        queue.push(std::move(msgCopy));
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

SampleSetDirectoryConverter::SampleSetDirectoryConverter(
    const std::filesystem::path &path)
    : directoryPath(path), running(false) {}

SampleSetDirectoryConverter::~SampleSetDirectoryConverter() {}

void SampleSetDirectoryConverter::process() {
  running = true;
  processInternal();
}

bool SampleSetDirectoryConverter::isRunning() { return running; }

void SampleSetDirectoryConverter::stop() { running = false; }

std::string topicNameForCamera(const std::string dirName) {
  std::string lowerDirName;
  std::transform(dirName.begin(), dirName.end(), std::back_inserter(lowerDirName),
                 ::tolower);
  return std::string("/") + lowerDirName + "/image";
}

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

  std::vector<std::pair<TopicInfo, TypeErasedQueue>> typeErasedQueueList;
  std::vector<std::unique_ptr<SampleSetDirectoryConverter>> sampleSetConverters;
  // Launch the pool with four threads.
  boost::asio::thread_pool pool(6);

  for (const auto &chosenSet : chosenSets) {
    if (chosenSet.descriptor.setType == SampleSetType::CAMERA) {

      auto queueProducerConsumerPair =
          SampleQueueFactory<sensor_msgs::Image>::makeQueue();

      // check if value is really moved here
      typeErasedQueueList.emplace_back(
          TopicInfo(topicNameForCamera(chosenSet.descriptor.directoryName)),
          TypeErasedQueue(queueProducerConsumerPair.second));

      sampleSetConverters.push_back(std::make_unique<ImageDirectoryConverter>(
          std::move(queueProducerConsumerPair.first), chosenSet.directoryPath));
      SampleSetDirectoryConverter* converter = sampleSetConverters.back().get();

      boost::asio::post(pool, [converter]() { converter->process(); });
    }
  }

  fs::remove(outputRosbagPath);

  MyProcessor processor(outputRosbagPath.filename());

  while (true) {
    bool atLeastOneQueueIsStillOpen = false;
    for (auto &[topicInfo, queue] : typeErasedQueueList) {
      if (!queue.isClosed() || (queue.size() > 0)) {
        atLeastOneQueueIsStillOpen = true;
        queue.process(topicInfo, processor);
      }
    }
    if (!atLeastOneQueueIsStillOpen) {
      std::cout << "completed all queue" << std::endl;
      break;
    }
  }

  pool.join();
  // std_msgs::String str;
  // str.data = std::string("foo");

  // std_msgs::Int32 i;
  // i.data = 42;

}