#include "nuscenes2bag/NuScenes2Bag.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/RunEvery.hpp"
#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include "nuscenes2bag/thread_pool.hpp"

#include <boost/make_shared.hpp>

#include <iostream>

#include <array>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <thread>

using namespace std;

namespace nuscenes2bag {

NuScenes2Bag::NuScenes2Bag() {}

void
NuScenes2Bag::convertDirectory(const fs::path& inDatasetPath,
                               const std::string& version,
                               const fs::path& outputRosbagPath,
                               int threadNumber,
#if CMAKE_CXX_STANDARD >= 17
                               std::optional<int32_t> sceneNumberOpt
#else
                               int32_t sceneNumber
#endif
                               )
{
  if ((threadNumber < 1) || (threadNumber > 64)) {
    std::cout << "Forcing at least one job number (-j1)" << std::endl;
    threadNumber = 1;
  }

  MetaDataReader metaDataReader;

  fs::path metadataPath = inDatasetPath;
  metadataPath /= fs::path(version); // Append sub-directory
  std::cout << "Loading metadata from " + metadataPath.string() + " ..." << std::endl;

  try {
    // If file is not found, a runtime_error is thrown
  metaDataReader.loadFromDirectory(metadataPath);
  } catch (const runtime_error& e) {
      std::cerr << "Error: " << e.what() << '\n';
      std::exit(-1);
  }

  cout << "Initializing " << threadNumber << " threads..." << endl;

  ThreadPool<FIFO> pool(threadNumber);
  std::vector<boost::shared_ptr<SceneConverter>> sceneConverters;

  FileProgress fileProgress;

  fs::create_directories(outputRosbagPath);

  std::vector<Token> chosenSceneTokens;

  if (sceneNumber > 0) {
    boost::shared_ptr<SceneInfo> sceneInfo = metaDataReader.getSceneInfoByNumber(sceneNumber);
    if(sceneInfo) {
      std::cout << "Found scene number: " << sceneNumber
                << "  name: " << sceneInfo->name
                << std::endl;
      chosenSceneTokens.push_back(sceneInfo->token);
    } else {
      std::cout << "Scene with ID=" << sceneNumber << " not found!" << std::endl;
    }
  } else {
    chosenSceneTokens = metaDataReader.getAllSceneTokens();
    std::cout << "Found " << chosenSceneTokens.size() << " scenes in directory" << std::endl;
  }

  int counter = 0;

  for (const auto& sceneToken : chosenSceneTokens) {
    boost::shared_ptr<SceneConverter> sceneConverter = boost::make_shared<SceneConverter>(SceneConverter(metaDataReader));
    sceneConverter->submit(sceneToken, fileProgress);
    sceneConverters.push_back(std::move(sceneConverter));

    // Add task to FIFO queue.
    // If we use 4 threads then we finish converting 4 scenes to bag files
    // before starting to convert the 5th.
    auto fn1 = [&, sceneConverters]()
    {
      auto sceneInfo = metaDataReader.getSceneInfo(sceneToken);
      std::cout << "Converting log " << counter << " of " << chosenSceneTokens.size() << ", " << sceneInfo->name << std::endl;
      sceneConverters.back()->run(inDatasetPath, outputRosbagPath, fileProgress);
    };
    pool.enqueue(fn1);

    counter++;
  }

  while (fileProgress.processedFiles != fileProgress.toProcessFiles) {
    std::cout << "Progress: "
              << static_cast<int>(fileProgress.getProgressPercentage() * 100)
              << "% [" << fileProgress.processedFiles << "/"
              << fileProgress.toProcessFiles << "]" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

}

}

