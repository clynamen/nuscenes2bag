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

// "token": "5d238c9840c64e6188bba20a2917e48e",
// "timestamp": 1532402945801309,
// "rotation": [
// 0.9858272960625895,
// -0.02308945425603873,
// 0.009966776080608072,
// -0.16586766657380866
// ],
// "translation": [
// 418.3997186638232,
// 1105.1880827769266,
// 0.0
// ]

struct EgoPoseInfo {
    Token token;
    TimeStamp timeStamp;
    double rotation[4];
    double translation[4];
};

template <> std::string to_debug_string(const SceneInfo& t);
template <> std::string to_debug_string(const SampleInfo& t);
template <> std::string to_debug_string(const SampleDataInfo& t);