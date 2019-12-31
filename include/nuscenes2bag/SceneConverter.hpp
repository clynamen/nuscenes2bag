#pragma once

#include "nuscenes2bag/Filesystem.hpp"
#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/FileProgress.hpp"
#include "rosbag/bag.h"

namespace nuscenes2bag {

class SceneConverter {
    public:
    SceneConverter(const MetaDataProvider& metaDataProvider);

    void submit(const Token& sceneToken, FileProgress& fileProgress);

    void run(const fs::path& inPath, const fs::path& outDirectoryPath, FileProgress& fileProgress);

    private:
    void convertSampleDatas(rosbag::Bag& outBag, const fs::path &inPath, FileProgress& fileProgress);
    void convertEgoPoseInfos(rosbag::Bag& outBag, const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfo);
    
    private:
    const MetaDataProvider& metaDataProvider;
    std::vector<SampleDataInfo> sampleDatas;
    std::vector<EgoPoseInfo> egoPoseInfos;
    SceneId sceneId;
    Token sceneToken;
};

}