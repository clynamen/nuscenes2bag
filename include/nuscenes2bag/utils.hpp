#pragma once

#include "nuscenes2bag/DatasetTypes.hpp"

#include "ros/ros.h"
#include <string>

std::string toLower(const std::string_view &str);

bool string_icontains(const std::string_view &string,
                      const std::string_view &sub);

ros::Time stampUs2RosTime(uint64_t stampUs);

std::string topicNameForSampleType(const std::string dirName,
                                      const SampleType SampleType);

std::string topicNameDefault(const std::string &dirName);

std::string topicNameForCamera(const std::string &dirName);

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
