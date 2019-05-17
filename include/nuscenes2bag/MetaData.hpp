#pragma once 

#include <string>

#include "nuscenes2bag/ToDebugString.hpp"

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
    Token scene_token;
    Token token;
    TimeStamp timeStamp;
};

struct SampleDataInfo {
    // Token scene_token;
    Token token;
    TimeStamp timeStamp;
    Token egoPoseToken;
    Token calibratedSensorToken;
    std::string fileFormat;
    bool isKeyFrame;
    std::string fileName;
};

struct EgoPoseInfo {
    Token token;
    TimeStamp timeStamp;
    double translation[3];
    double rotation[4];
};

template <> std::string to_debug_string(const SceneInfo& t);
template <> std::string to_debug_string(const SampleInfo& t);
template <> std::string to_debug_string(const SampleDataInfo& t);
