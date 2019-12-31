#include "nuscenes2bag/NuScenes2Bag.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/RunEvery.hpp"
#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/utils.hpp"

#include <memory> // std::unique_ptr

#include <boost/asio.hpp>

#if BOOST_VERSION >= 106600
// thread_pool was added in Boost 1.66.0
#include <boost/asio/thread_pool.hpp>
#else
#include "nuscenes2bag/thread_pool.hpp"
#endif

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
                               boost::optional<int32_t> sceneNumberOpt)
{
  if ((threadNumber < 1) || (threadNumber > 64)) {
    std::cout << "Forcing at least one job number (-j1)" << std::endl;
    threadNumber = 1;
  }

  MetaDataReader metaDataReader;

  fs::path metadataPath = inDatasetPath;
  metadataPath /= fs::path(version); // Append sub-directory
  std::cout << "Loading metadata from " + metadataPath.string() + " ..."
            << std::endl;

  try {
    // If file is not found, a runtime_error is thrown
    metaDataReader.loadFromDirectory(metadataPath);
  } catch (const runtime_error& e) {
    std::cerr << "Error: " << e.what() << '\n';
    std::exit(-1);
  }

  cout << "Initializing " << threadNumber << " threads..." << endl;

#if BOOST_VERSION >= 106600
  boost::asio::thread_pool pool(threadNumber);
  std::vector<std::unique_ptr<SceneConverter>> sceneConverters;
#else
  ThreadPool<FIFO> pool(threadNumber);
  std::vector<boost::shared_ptr<SceneConverter>> sceneConverters;
#endif

  FileProgress fileProgress;

  fs::create_directories(outputRosbagPath);

  std::vector<Token> chosenSceneTokens;

#if BOOST_VERSION >= 106600

  if (sceneNumberOpt) {
    auto sceneInfoOpt =
      metaDataReader.getSceneInfoByNumber(sceneNumberOpt.value());
    if (sceneInfoOpt) {
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

  RunEvery<std::function<void()>> showProgress(
    std::chrono::milliseconds(1000), [&fileProgress]() {
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

#else

  if (sceneNumberOpt) {
    auto sceneInfoOpt =
      metaDataReader.getSceneInfoByNumber(sceneNumberOpt.value());
    if (sceneInfoOpt) {
      chosenSceneTokens.push_back(sceneInfoOpt->token);
    } else {
      std::cout << "Scene with ID=" << sceneNumberOpt.value() << " not found!"
                << std::endl;
    }
  } else {
    chosenSceneTokens = metaDataReader.getAllSceneTokens();
    ;
  }

  int counter = 0;

  for (const auto& sceneToken : chosenSceneTokens) {
    boost::shared_ptr<SceneConverter> sceneConverter =
      boost::make_shared<SceneConverter>(SceneConverter(metaDataReader));
    sceneConverter->submit(sceneToken, fileProgress);
    sceneConverters.push_back(std::move(sceneConverter));

    // Add task to FIFO queue.
    // If we use 4 threads then we finish converting 4 scenes to bag files
    // before starting to convert the 5th.
    auto fn1 = [&, sceneConverters]() {
      auto sceneInfo = metaDataReader.getSceneInfo(sceneToken);
      std::cout << "Converting log " << counter << " of "
                << chosenSceneTokens.size() << ", " << sceneInfo->name
                << std::endl;
      sceneConverters.back()->run(
        inDatasetPath, outputRosbagPath, fileProgress);
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

#endif
}

}
