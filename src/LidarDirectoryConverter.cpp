#include "nuscenes2bag/LidarDirectoryConverter.hpp"

using namespace sensor_msgs;
using namespace std;

std::optional<sensor_msgs::PointCloud2>
readLidarFile(std::filesystem::path filePath)
{

  PointCloud2 cloud;

  try {
    cloud.header.frame_id = std::string("lidar");
    cloud.is_bigendian = false;
    cloud.point_step = sizeof(uint32_t) * 4;
    cloud.row_step = 1;
    cloud.height = 1;

    std::vector<float> fileValues;
    std::ifstream fin(filePath.string(), std::ios::binary);
    float f;
    uint8_t skipCounter = 0;
    uint32_t floatRead = 0;
    while (fin.read(reinterpret_cast<char*>(&f), sizeof(float))) {
      floatRead++;
      if (skipCounter < 4) {
        fileValues.push_back(f);
        skipCounter++;
      } else {
        skipCounter = 0;
      }
    }

    uint32_t pointsRead = fileValues.size() / 4;

    cloud.width = fileValues.size() / 4;

    std::vector<uint8_t> data;
    // data.reserve(fileValues.size()*4);
    for (auto float_data : fileValues) {
      data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 0) & 0xFF);
      data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 8) & 0xFF);
      data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 16) & 0xFF);
      data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 24) & 0xFF);
    }

#define FLOAT32 7

    PointField field;
    field.datatype = FLOAT32;
    field.offset = 0;
    field.count = 1;
    field.name = std::string("x");
    cloud.fields.push_back(field);

    field.datatype = FLOAT32;
    field.offset = 4;
    field.count = 1;
    field.name = std::string("y");
    cloud.fields.push_back(field);

    field.datatype = FLOAT32;
    field.offset = 8;
    field.count = 1;
    field.name = std::string("z");
    cloud.fields.push_back(field);

    field.datatype = FLOAT32;
    field.offset = 12;
    field.count = 1;
    field.name = std::string("intensity");
    cloud.fields.push_back(field);

    cloud.data = data;

  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    return std::nullopt;
  }

  return std::optional(cloud);
}
