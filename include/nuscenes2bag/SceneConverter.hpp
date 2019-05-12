#pragma once

#include <filesystem>

#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/FileProgress.hpp"
#include "rosbag/bag.h"

class SceneConverter {
    public:
    SceneConverter(const MetaDataProvider& metaDataProvider);

    void submit(const Token& sceneToken, FileProgress& fileProgress);
    void run(const std::filesystem::path& inPath, const std::filesystem::path& outDirectoryPath, FileProgress& fileProgress);
    
    private:
    const MetaDataProvider& metaDataProvider;
    std::vector<SampleDataInfo> sampleDatas;
    SceneId sceneId;
};