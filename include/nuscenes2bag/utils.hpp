#pragma once

#include "nuscenes2bag/DatasetTypes.hpp"

#include "ros/ros.h"
#include <string>
#include <iostream>
#include <exception>

#define PRINT_EXCEPTION(e) std::cout << "[ERROR] Exception thrown: " << __FILE__ << ":" << __LINE__ << " " << e.what() << std::endl;

namespace nuscenes2bag {

std::string toLower(const std::string_view &str);

bool string_icontains(const std::string_view &string,
                      const std::string_view &sub);

ros::Time stampUs2RosTime(uint64_t stampUs);

template <class T> T uniq(T t) {
  sort(t.begin(), t.end());
  t.erase(unique(t.begin(), t.end()), t.end());
  return t;
}

template <template <class, class, class...> class Container, class Key,
          class Value, class... TArgs>
Value &getExistingOrDefault(Container<Key, Value, TArgs...> &container,
                            const Key &key) {
  auto it = container.find(key);
  if (it == container.end()) {
    container.insert(std::pair<Key, Value>(key, Value()));
    it = container.find(key);
    return it->second;
  } else {
    return it->second;
  }
}

template <typename T, typename U> void assignArray2Vector3(T& vector3, const U* ar) {
    vector3.x = ar[0];
    vector3.y = ar[1];
    vector3.z = ar[2];
}

template <typename T, typename U> void assignArray2Quaternion(T& quat, const U* ar) {
    quat.x = ar[1];
    quat.y = ar[2];
    quat.z = ar[3];
    quat.w = ar[0];
}

class UnableToParseFileException : public std::exception {
  private:
    std::string msg;

  public:
    UnableToParseFileException(const std::string& fileName) {
      msg += "Unable to parse ";
      msg += fileName;
    };
    ~UnableToParseFileException() throw() {};
    const char *what() const throw() { return this->msg.c_str(); };
};

}