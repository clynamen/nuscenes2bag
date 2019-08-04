#include "nuscenes2bag/NuScenes2Bag.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/RunEvery.hpp"
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

namespace nuscenes2bag {

NuScenes2Bag::NuScenes2Bag() {}

void
NuScenes2Bag::convertDirectory(const std::filesystem::path& inMetaPath,
                               const std::filesystem::path& inDatasetPath,
                               const std::filesystem::path& outputRosbagPath,
                               int threadNumber,
                               std::optional<int32_t> sceneNumberOpt)
{
  if (threadNumber < 1) {
    std::cout << "Forcing at least one job number (-j1)" << std::endl;
    threadNumber = 1;
  } else {
    cout << "Using " << threadNumber << " threads" << endl;
  }

  MetaDataReader metaDataReader;
  cout << "Loading metadata from " << inMetaPath  << endl;
  metaDataReader.loadFromDirectory(inMetaPath);

  cout << "Creating threads " << endl;
  boost::asio::thread_pool pool(threadNumber);
  std::vector<std::unique_ptr<SceneConverter>> sceneConverters;
  FileProgress fileProgress;

  cout << "Create out directory " << endl;
  fs::create_directories(outputRosbagPath);

  std::vector<Token> chosenSceneTokens;

  if (sceneNumberOpt.has_value()) {
    auto sceneInfoOpt =
      metaDataReader.getSceneInfoByNumber(sceneNumberOpt.value());
    if (sceneInfoOpt.has_value()) {
      chosenSceneTokens.push_back(sceneInfoOpt->token);
    } else {
      std::cout << "Scene with ID=" << sceneNumberOpt.value() << " not found!"
                << std::endl;
    }
  } else {
    chosenSceneTokens = metaDataReader.getAllSceneTokens();
    ;
  }

  for (const auto& sceneToken : chosenSceneTokens) {
    std::unique_ptr<SceneConverter> sceneConverter =
      std::make_unique<SceneConverter>(metaDataReader);
    sceneConverter->submit(sceneToken, fileProgress);
    SceneConverter* sceneConverterPtr = sceneConverter.get();
    sceneConverters.push_back(std::move(sceneConverter));
    boost::asio::defer(pool, [&, sceneConverterPtr]() {
      sceneConverterPtr->run(inDatasetPath, outputRosbagPath, fileProgress);
    });
  }

  RunEvery showProgress(std::chrono::milliseconds(1000), [&fileProgress]() {
    std::cout << "Progress: "
              << static_cast<int>(fileProgress.getProgressPercentage() * 100)
              << "% [" << fileProgress.processedFiles << "/"
              << fileProgress.toProcessFiles << "]" << std::endl;
  });

  // TODO: replace check with futures
  while (fileProgress.processedFiles != fileProgress.toProcessFiles) {
    showProgress.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  pool.join();
}

}
