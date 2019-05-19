#pragma once 

#include <string>
#include <optional>
#include <array>

#include "nuscenes2bag/ToDebugString.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"

namespace nuscenes2bag {

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

struct CalibratedSensorInfo {
    Token token;
    Token sensorToken;
    double translation[3];
    double rotation[4];
    std::optional<IntrinsicsMatrix> cameraIntrinsics;
};

struct CalibratedSensorName {
    Token token;
    std::string name;
    std::string modality;
};

struct CalibratedSensorInfoAndName {
    CalibratedSensorInfo info;
    CalibratedSensorName name;

    inline friend bool operator<(const CalibratedSensorInfoAndName& l, const CalibratedSensorInfoAndName& r)
    {
        return l.info.token < r.info.token;
    }
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

}