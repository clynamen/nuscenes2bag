#include <iostream>
#include <filesystem>
#include <string>
#include <map>
#include "nuscenes2bag/ToDebugString.hpp"
#include <nlohmann/json.hpp>

typedef std::string Token;
typedef uint64_t TimeStamp;
typedef uint32_t SceneId;

struct SceneInfo {
    Token token; 
    uint32_t sampleNumber;
    SceneId sceneId;
    std::string name;
    std::string description;
    Token firstSampleToken; 
};

struct SampleInfo {
    Token token;
    TimeStamp timeStamp;
};

struct SampleDataInfo {
    Token token;
    TimeStamp timeStamp;
    Token egoPoseToken;
    Token calibratedSensorToken;
    std::string fileFormat;
    bool isKeyFrame;
    std::string fileName;
};



template <> std::string to_debug_string(const SceneInfo& t);

class MetaDataReader {
    public:
        void loadFromDirectory(const std::filesystem::path& directoryPath);

        static nlohmann::json slurpJsonFile(const std::filesystem::path& filePath);
        static std::vector<SceneInfo> loadScenesFromFile(const std::filesystem::path& filePath);
        static std::map<Token, std::vector<SampleInfo>> loadSampleInfos(const std::filesystem::path& filePath);
        static std::map<Token, std::vector<SampleDataInfo>> loadSampleDataInfos(const std::filesystem::path& filePath);


    private:
        std::vector<SceneInfo> scenes;
        std::map<Token, std::vector<SampleInfo>> scene2Samples;
        std::map<Token, std::vector<SampleDataInfo>> sample2SampleData;
};