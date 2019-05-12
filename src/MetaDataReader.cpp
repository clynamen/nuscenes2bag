#include <nuscenes2bag/MetaDataReader.hpp>
#include "nuscenes2bag/utils.hpp"

#include <fstream>
#include <iostream>
#include <filesystem>
#include <map>

using namespace std;
namespace fs = std::filesystem;
namespace json = nlohmann;


template <> std::string to_debug_string(const SceneInfo& s) {
    std::ostringstream os;
    os << "{" << "token: " << s.token
      << ", sampleNumber: " << s.sampleNumber
      << ", sceneId: " << s.sceneId
      << ", name: " << s.name
      << ", description: " << s.description
    << "}";
    return os.str(); 
}

void MetaDataReader::loadFromDirectory(const fs::path& directoryPath) {
    const fs::path sceneFile = directoryPath / "scene.json";
    const fs::path sampleFile = directoryPath / "sample.json";
    const fs::path sampleDataFile = directoryPath / "sample_data.json";

    scenes = loadScenesFromFile(sceneFile);
    scene2Samples = loadSampleInfos(sampleFile);
    sample2SampleData = loadSampleDataInfos(sampleDataFile);

    // for(const auto& scene: scenes) {
    //     cout << to_debug_string(scene) << endl;
    // }


}


json::json MetaDataReader::slurpJsonFile(const std::filesystem::path& filePath) {
    std::ifstream file(filePath.string());
    if(!file.is_open()) {
        std::string errMsg = string("Unable to open ") + filePath.string();
        throw std::runtime_error(errMsg);
    }
    json::json newJson;
    file >> newJson;
    return newJson;
}

std::vector<SceneInfo> MetaDataReader::loadScenesFromFile(const fs::path& filePath) {
    auto sceneJsons = slurpJsonFile(filePath);
    std::vector<SceneInfo> sceneInfos;

    for(const auto& sceneJson : sceneJsons) {
        sceneInfos.push_back(SceneInfo{
            sceneJson["token"],
            sceneJson["nbr_samples"],
            0,
            sceneJson["name"],
            sceneJson["description"],
            sceneJson["first_sample_token"],
        });
    }

    return sceneInfos;
}

std::map<Token, std::vector<SampleInfo>> MetaDataReader::loadSampleInfos(const std::filesystem::path& filePath) {
    auto sampleInfos = slurpJsonFile(filePath);
    std::map<Token, std::vector<SampleInfo>> token2Samples;

    for(const auto& sampleInfo : sampleInfos) {
        Token sampleToken = sampleInfo["token"];
        std::vector<SampleInfo>& samples = getExistingOrDefault(token2Samples, sampleToken);
        samples.push_back(SampleInfo{
            sampleToken, 
            sampleInfo["timestamp"]
        });
    }

    return token2Samples;
}

std::map<Token, std::vector<SampleDataInfo>> MetaDataReader::loadSampleDataInfos(const std::filesystem::path& filePath) {
    auto sampleDataJsons = slurpJsonFile(filePath);
    std::map<Token, std::vector<SampleDataInfo>> sample2SampleData;

    for(const auto& sampleDataJson : sampleDataJsons) {
        Token sampleToken = sampleDataJson["token"];
        std::vector<SampleDataInfo>& sampleDatas = getExistingOrDefault(sample2SampleData, sampleToken);
        sampleDatas.push_back(SampleDataInfo{
            sampleToken,
            sampleDataJson["timestamp"],
            sampleDataJson["ego_pose_token"],
            sampleDataJson["calibrated_sensor_token"],
            sampleDataJson["fileformat"],
            sampleDataJson["is_key_frame"],
            sampleDataJson["filename"],
        });
    }

    return sample2SampleData;
}
