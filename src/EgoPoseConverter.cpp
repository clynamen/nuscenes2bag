#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include "nuscenes2bag/utils.hpp"
#include <nuscenes2bag/MetaData.hpp>

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

nav_msgs::Odometry egoPoseInfo2OdometryMsg(const EgoPoseInfo &egoPoseInfo) {
  nav_msgs::Odometry msg;
  msg.header.stamp = stampUs2RosTime(egoPoseInfo.timeStamp);
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  assignArray2Vector3(msg.pose.pose.position, egoPoseInfo.translation);
  assignArray2Quaternion(msg.pose.pose.orientation, egoPoseInfo.rotation);

  return msg;
}

geometry_msgs::TransformStamped egoPoseInfo2TransformStamped(const EgoPoseInfo &egoPoseInfo) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stampUs2RosTime(egoPoseInfo.timeStamp);
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  assignArray2Vector3(msg.transform.translation, egoPoseInfo.translation);
  assignArray2Quaternion(msg.transform.rotation, egoPoseInfo.rotation);

  return msg;
}