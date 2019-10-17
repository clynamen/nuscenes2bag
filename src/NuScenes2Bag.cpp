#include "nuscenes2bag/NuScenes2Bag.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/RunEvery.hpp"
#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/utils.hpp"

#if CMAKE_CXX_STANDARD >= 17
// We use std::filesystem and std::optional from C++17, and std::unique_ptr from C++14
#include <memory> // std::unique_ptr
#else
#include <boost/make_shared.hpp>
#endif

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
  cout << "Loading metadata..." << endl;

  try {
    // If file is not found, a runtime_error is thrown
  metaDataReader.loadFromDirectory(inDatasetPath);
  } catch (const runtime_error& e) {
      std::cerr << "Error: " << e.what() << '\n';
      std::exit(-1);
  }

  cout << "Initializing " << threadNumber << " threads..." << endl;

#if (CMAKE_CXX_STANDARD >= 17) && (BOOST_VERSION >= 106600)
  boost::asio::thread_pool pool(threadNumber);
  std::vector<std::unique_ptr<SceneConverter>> sceneConverters;
#else
  ThreadPool<FIFO> pool(threadNumber);
  std::vector<boost::shared_ptr<SceneConverter>> sceneConverters;
#endif

  FileProgress fileProgress;

  fs::create_directories(outputRosbagPath);

  std::vector<Token> chosenSceneTokens;

#if (CMAKE_CXX_STANDARD >= 17) && (BOOST_VERSION >= 106600)

  if(sceneNumberOpt.has_value()) {
    auto sceneInfoOpt = metaDataReader.getSceneInfoByNumber(sceneNumberOpt.value());
    if(sceneInfoOpt.has_value()) {
      chosenSceneTokens.push_back(sceneInfoOpt->token);
    } else {
      std::cout << "Scene with ID=" << sceneNumberOpt.value() << " not found!" << std::endl;
    }
  } else {
    chosenSceneTokens = metaDataReader.getAllSceneTokens();;
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

#else

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

#endif

}

}

