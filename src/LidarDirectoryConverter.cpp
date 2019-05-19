#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <exception>

using namespace sensor_msgs;
using namespace std;

namespace nuscenes2bag {

constexpr uint32_t FLOAT32 = 7;

inline void fillFieldsForPointcloud(std::vector<PointField>& fields) {
    PointField field;
    field.datatype = FLOAT32;
    field.offset = 0;
    field.count = 1;
    field.name = std::string("x");
    fields.push_back(field);

    field.datatype = FLOAT32;
    field.offset = 4;
    field.count = 1;
    field.name = std::string("y");
    fields.push_back(field);

    field.datatype = FLOAT32;
    field.offset = 8;
    field.count = 1;
    field.name = std::string("z");
    fields.push_back(field);

    field.datatype = FLOAT32;
    field.offset = 12;
    field.count = 1;
    field.name = std::string("intensity");
    fields.push_back(field);
}

inline void push_back_float32(std::vector<uint8_t>& data, float float_data) {
  data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 0) & 0xFF);
  data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 8) & 0xFF);
  data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 16) & 0xFF);
  data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 24) & 0xFF);
}

inline std::vector<float> readBinaryPcdFile(std::ifstream& fin) {
  std::vector<float> fileValues;
  uint8_t skipCounter = 0;
  float f;
  while (fin.read(reinterpret_cast<char*>(&f), sizeof(float))) {
    // skip 5th value of each point
    if (skipCounter < 4) {
      fileValues.push_back(f);
      skipCounter++;
    } else {
      skipCounter = 0;
    }
  }

  return fileValues;
}

std::optional<sensor_msgs::PointCloud2>
readLidarFile(const std::filesystem::path& filePath)
{

  PointCloud2 cloud;
  cloud.header.frame_id = std::string("lidar");
  cloud.is_bigendian = false;
  cloud.point_step = sizeof(uint32_t) * 4;
  cloud.row_step = 1;
  cloud.height = 1;

  try {
    std::ifstream fin(filePath.string(), std::ios::binary);
    const std::vector<float> fileValues = readBinaryPcdFile(fin);

    if(fileValues.size() % 4 != 0) {
      throw UnableToParseFileException(filePath.string());
    }
    const size_t pointsNumber = fileValues.size() / 4;
    cloud.width = pointsNumber;

    std::vector<uint8_t> data;
    for (auto float_data : fileValues) {
      push_back_float32(data, float_data);
    }

    fillFieldsForPointcloud(cloud.fields);
    cloud.data = data;

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);
    return std::nullopt;
  }

  return std::optional(cloud);
}

}