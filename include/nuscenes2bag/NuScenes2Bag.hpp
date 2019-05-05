#pragma once

#include <rosbag/bag.h>

#include "nuscenes2bag/SampleSetDescriptor.hpp"

#include <filesystem>
#include <optional>
#include <vector>

struct NuScenes2Bag {

public:
  NuScenes2Bag();

  void convertDirectory(const std::filesystem::path &inDatasetPath,
                        const std::filesystem::path &outputRosbagPath);

  std::vector<SceneId> getSceneIdsInDirectory(
      const std::filesystem::path &inDirectoryPath);

  std::vector<FileSystemSampleSet> extractSampleSetsDescriptorInDirectory(
      const std::filesystem::path &inDirectoryPath);

  std::vector<FileSystemSampleSet>
  getSampleSetsInDirectory(const std::filesystem::path &inDatasetPath);

  std::vector<FileSystemSampleSet>
  filterChosenSampleSets(const std::vector<FileSystemSampleSet> &sampleSet);

  void processSampleSets(
      const std::vector<FileSystemSampleSet> &sampleSets,
      const std::filesystem::path &outputRosbagPath);

private:
  std::string inDatasetPathString;
};