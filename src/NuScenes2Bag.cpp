#include "nuscenes2bag/NuScenes2Bag.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/MyProcessor.hpp"
#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/RunEvery.hpp"
#include "nuscenes2bag/SampleQueue.hpp"
#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/utils.hpp"

#include <boost/asio.hpp>

#include <iostream>

#include <array>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <thread>

using namespace std;
namespace fs = std::filesystem;

NuScenes2Bag::NuScenes2Bag() {}

std::vector<SceneId> NuScenes2Bag::getSceneIdsInDirectory(
    const std::filesystem::path &inDirectoryPath) {
  std::vector<SceneId> sceneIds;
  const auto fileSamples = getFilesInSampleSetDirectory(inDirectoryPath);

  std::transform(
      fileSamples.begin(), fileSamples.end(), std::back_inserter(sceneIds),
      [](const auto &fileSample) { return fileSample.second.sceneId; });

  sceneIds = uniq(sceneIds);

  return sceneIds;
}

std::vector<FileSystemSampleSet>
NuScenes2Bag::extractSampleSetsDescriptorInDirectory(
    const std::filesystem::path &inDirectoryPath) {
  const std::string &dirName = inDirectoryPath.filename();

  struct InfoPreset {
    const char *name;
    const SampleSetType sampleType;
  };

  static std::array<InfoPreset, 3> presets{
      "CAM",   SampleSetType::CAMERA, "LIDAR", SampleSetType::LIDAR,
      "RADAR", SampleSetType::RADAR,
  };

  std::vector<FileSystemSampleSet> sampleSets;

  for (auto &preset : presets) {
    if (string_icontains(dirName, preset.name)) {
      auto sceneIds = getSceneIdsInDirectory(inDirectoryPath);
      // std::cout << "All sceneIds " << std::endl;
      // for(auto& sceneId: sceneIds) {
      //   std::cout << "id: " << sceneId << std::endl;
      // }
      std::transform(sceneIds.begin(), sceneIds.end(),
                     std::back_inserter(sampleSets), [&](const auto sceneId) {
                       return FileSystemSampleSet{
                           {sceneId, dirName, preset.sampleType},
                           inDirectoryPath};
                     });
      break;
    } else {
      continue;
    }
  }

  if (sampleSets.empty()) {
    std::cout << "No sample set found in " << inDirectoryPath << std::endl;
  }

  return sampleSets;
}

std::vector<FileSystemSampleSet> NuScenes2Bag::getSampleSetsInDirectory(
    const std::filesystem::path &inDatasetPath) {
  std::vector<FileSystemSampleSet> sets;

  for (auto &dir : fs::directory_iterator(inDatasetPath)) {
    if (dir.is_directory()) {
      auto newSampleSet = extractSampleSetsDescriptorInDirectory(dir.path());
      sets.insert(sets.end(), newSampleSet.begin(), newSampleSet.end());
    }
  }

  return sets;
}

std::vector<FileSystemSampleSet> NuScenes2Bag::filterChosenSampleSets(
    const std::vector<FileSystemSampleSet> &sampleSets) {
  std::vector<FileSystemSampleSet> filteredSets;
  std::copy_if(sampleSets.begin(), sampleSets.end(),
               std::back_inserter(filteredSets), [](auto &fileSystemSampleSet) {
                 return fileSystemSampleSet.descriptor.setType ==
                        SampleSetType::RADAR;
                 // return true;
               });

  return filteredSets;
}

template <typename T>
void addNewConverter(
    const FileSystemSampleSet &sampleSet,
    std::vector<std::pair<TopicInfo, TypeErasedQueue>> &typeErasedQueueList,
    std::vector<std::unique_ptr<SampleSetDirectoryConverter>>
        &sampleSetConverters,
    FileProgress &fileProgress) {
  auto queueProducerConsumerPair = SampleQueueFactory<T>::makeQueue();
  typeErasedQueueList.emplace_back(
      TopicInfo{sampleSet.descriptor.sceneId,
                topicNameForSampleSetType(sampleSet.descriptor.directoryName,
                                          sampleSet.descriptor.setType)},
      TypeErasedQueue(queueProducerConsumerPair.second));
  sampleSetConverters.push_back(std::make_unique<MsgDirectoryConverter<T>>(
      std::move(queueProducerConsumerPair.first), fileProgress,
      sampleSet.directoryPath));
}

void NuScenes2Bag::processSampleSets(
    const std::vector<FileSystemSampleSet> &sampleSets,
    const std::filesystem::path &outputRosbagPath) {

  std::vector<std::pair<TopicInfo, TypeErasedQueue>> typeErasedQueueList;
  std::vector<std::unique_ptr<SampleSetDirectoryConverter>> sampleSetConverters;

  // Launch the pool with four threads.
  boost::asio::thread_pool pool(2);

  FileProgress fileProgress;

  for (const auto &sampleSet : sampleSets) {

    if (sampleSet.descriptor.setType == SampleSetType::CAMERA) {
      addNewConverter<sensor_msgs::Image>(sampleSet, typeErasedQueueList,
                                          sampleSetConverters, fileProgress);
    } else if (sampleSet.descriptor.setType == SampleSetType::LIDAR) {
      addNewConverter<sensor_msgs::PointCloud2>(
          sampleSet, typeErasedQueueList, sampleSetConverters, fileProgress);
    } else if (sampleSet.descriptor.setType == SampleSetType::RADAR) {
      addNewConverter<nuscenes2bag::RadarObjects>(
          sampleSet, typeErasedQueueList, sampleSetConverters, fileProgress);
    } else {
      throw std::runtime_error("Not supported SampleSetType");
    }

    SampleSetDirectoryConverter *converter = sampleSetConverters.back().get();
    converter->submit();
    boost::asio::post(pool, [converter]() { converter->process(); });
  }

  MyProcessor processor(outputRosbagPath.filename());

  RunEvery showProgress(std::chrono::milliseconds(1000), [&fileProgress]() {
    std::cout << "Progress: " << fileProgress.getProgressPercentage() << " ["
              << fileProgress.processedFiles << "/"
              << fileProgress.toProcessFiles << "]" << std::endl;
  });

  while (true) {
    bool atLeastOneQueueIsStillOpen = false;
    for (auto &[topicInfo, queue] : typeErasedQueueList) {
      if (!queue.isClosed() || (queue.size() > 0)) {
        atLeastOneQueueIsStillOpen = true;
        queue.process(topicInfo, processor);
        showProgress.update();
      }
    }
    if (!atLeastOneQueueIsStillOpen) {
      std::cout << "completed all queue" << std::endl;
      break;
    }
  }

  pool.join();
}

void NuScenes2Bag::convertDirectory(
    const std::filesystem::path &inDatasetPath,
    const std::filesystem::path &outputRosbagPath) {

  auto availableSampleSets = getSampleSetsInDirectory(inDatasetPath);
  // std::cout << "Found " << availableSampleSets.size()
  //           << " valid sample directory" << std::endl;

  auto chosenSets = filterChosenSampleSets(availableSampleSets);
  // std::cout << "Chosen " << chosenSets.size() << " sample directory"
  //           << std::endl;

  for (auto &set : chosenSets) {
    // std::cout << to_debug_string(set) << std::endl;
  }

  processSampleSets(chosenSets, outputRosbagPath);
}

void NuScenes2Bag::convertDirectory2(
    const std::filesystem::path &inDatasetPath,
    const std::filesystem::path &outputRosbagPath) {

  MetaDataReader metaDataReader;
  cout << "Loading metadata..." << endl;
  metaDataReader.loadFromDirectory(inDatasetPath / "v1.0-mini");

  boost::asio::thread_pool pool;
  std::vector<std::unique_ptr<SceneConverter>> sceneConverters;
  FileProgress fileProgress;

  fs::create_directories(outputRosbagPath);
  for (const auto &sceneToken : metaDataReader.getAllSceneTokens()) {
    std::unique_ptr<SceneConverter> sceneConverter =
        std::make_unique<SceneConverter>(metaDataReader);
    sceneConverter->submit(sceneToken, fileProgress);
    SceneConverter* sceneConverterPtr = sceneConverter.get();
    sceneConverters.push_back(std::move(sceneConverter));
    boost::asio::defer(pool, [&, sceneConverterPtr]() {
      sceneConverterPtr->run(inDatasetPath, outputRosbagPath, fileProgress);
    });
    // std::cout << to_debug_string(set) << std::endl;
  }

  RunEvery showProgress(std::chrono::milliseconds(1000), [&fileProgress]() {
    std::cout << "Progress: " << fileProgress.getProgressPercentage() << " ["
              << fileProgress.processedFiles << "/"
              << fileProgress.toProcessFiles << "]" << std::endl;
  });

  // TODO: replace check with futures
  while(fileProgress.processedFiles != fileProgress.toProcessFiles) {
      showProgress.update();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  pool.join();
}