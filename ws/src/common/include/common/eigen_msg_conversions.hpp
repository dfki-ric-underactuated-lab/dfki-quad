#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

void eigenToRosMsg(const Eigen::Quaterniond& eigen_quat, geometry_msgs::msg::Quaternion& msg) {
  msg.x = eigen_quat.x();
  msg.y = eigen_quat.y();
  msg.z = eigen_quat.z();
  msg.w = eigen_quat.w();
}

template <typename T>
void eigenToRosMsg(const Eigen::Vector3d& eigen_vector, T& msg) {
  msg.x = eigen_vector.x();
  msg.y = eigen_vector.y();
  msg.z = eigen_vector.z();
}

template <typename T, std::size_t len>
void eigenToRosMsg(const std::array<Eigen::Vector3d, len>& eigen_vector, std::array<T, len>& msg) {
  for (std::size_t i = 0; i < len; i++) {
    msg[i].x = eigen_vector[i].x();
    msg[i].y = eigen_vector[i].y();
    msg[i].z = eigen_vector[i].z();
  }
}

template <typename T>
Eigen::Vector3d rosMsgToEigen(const T& msg) {
  Eigen::Vector3d vec;
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
  return vec;
}

template <typename outT, typename inT>
outT rosMsgToEigen(const inT& msg) {
  outT vec;
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
  return vec;
}

template <typename T>
T eigenToRosMsg(const Eigen::Vector3d& eigen_vector) {
  T msg;
  eigenToRosMsg(eigen_vector, msg);
  return msg;
}

template <typename>
struct is_std_vector : std::false_type {};

template <typename T, typename A>
struct is_std_vector<std::vector<T, A>> : std::true_type {};

template <typename outT, typename inT>
outT rosMsgSeqToEigenSeq(const inT& in) {
  outT out;
  if (is_std_vector<outT>::value) {
    out.resize(in.size());
  }
  assert(in.size() == out.size());
  for (std::size_t i = 0; i < in.size(); i++) {
    out[i] = rosMsgToEigen<typename outT::value_type, typename inT::value_type>(in[i]);
  }
  return out;
}
