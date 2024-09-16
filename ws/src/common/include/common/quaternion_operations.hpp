#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

template <typename Skalar>
double map_to_pmpi(Skalar value) {
  double mapped_value = fmod(value, (2 * M_PI));
  if (mapped_value > M_PI) {
    mapped_value -= 2 * M_PI;
  }
  // mapped_value -= M_PI;
  return mapped_value;
}

template <typename Skalar>
inline Eigen::Quaternion<Skalar> euler_to_quaternion(
    const Eigen::Ref<const Eigen::Vector<Skalar, 3>>& euler) {  // ZYX euler angle in order roll pitch yaw
  return Eigen::AngleAxis<Skalar>(euler(2), Eigen::Vector<Skalar, 3>::UnitZ())
         * Eigen::AngleAxis<Skalar>(euler(1), Eigen::Vector<Skalar, 3>::UnitY())
         * Eigen::AngleAxis<Skalar>(euler(0), Eigen::Vector<Skalar, 3>::UnitX());
}

template <typename Skalar>
inline Eigen::Vector<Skalar, 3> quaternion_to_euler(
    const Eigen::Quaternion<Skalar>& quat, int i, int j, int k, bool extrinsic = false) {
  // from: https://amu.hal.science/hal-03848730/document
  // TODO: parts of this code in combination with QuadModelSymbolic require the compiler optomizations to not be -Ofast
  // -> figure out why we get NaN then.

  // WARNING: NOT WORKING FOR NOW!!!!!!!!

  if (!extrinsic) {
    // SWAP
    int kn = i;
    int in = k;
    i = in;
    k = kn;
  }

  bool not_proper;
  Eigen::Vector<Skalar, 4> q{quat.x(), quat.y(), quat.z(), quat.w()};  // Has to be done due to storage order
  if (i == k) {
    not_proper = false;
    k = 3 - i - j;
  } else {
    not_proper = true;
  }
  int eps = (i - j) * (j - k) * (k - i) / 2;
  double a, b, c, d;
  if (not_proper) {
    a = q(3) - q(j);
    b = q(i) + q(k) * eps;
    c = q(j) + q(3);
    d = q(k) * eps - q(i);
  } else {
    a = q(3);
    b = q(i);
    c = q(j);
    d = q(k) * eps;
  }
  double theta2 = acos(2. * ((a * a + b * b) / (a * a + b * b + c * c + d * d)) - 1.);
  double thetap = atan2(b, a);
  double thetam = atan2(-d, c);
  double theta1, theta3;

  if ((-std::numeric_limits<double>::epsilon() <= theta2) and (theta2 <= std::numeric_limits<double>::epsilon())) {
    if (!extrinsic) {
      theta1 = 0;
      theta3 = 2 * thetap;
    } else {
      theta3 = 0;
      theta1 = 2 * thetap;
    }
  } else if ((M_PI_2 - std::numeric_limits<double>::epsilon() <= theta2)
             and (theta2 <= M_PI_2 + std::numeric_limits<double>::epsilon())) {
    if (!extrinsic) {
      theta1 = 0;
      theta3 = -2 * thetam;
    } else {
      theta3 = 0;
      theta1 = 2 * thetam;
    }
  } else {
    theta1 = thetap + thetam;
    theta3 = thetap - thetam;
  }

  // TODO: angle wrap

  if (not_proper) {
    theta3 = eps * theta3;
    theta2 = theta2 - M_PI_2;
  }

  if (!extrinsic) {
    return Eigen::Vector<Skalar, 3>{theta3, theta2, theta1};
  } else {
    return Eigen::Vector<Skalar, 3>{theta1, theta2, theta3};
  }
}

template <typename Skalar>
inline Eigen::Vector<Skalar, 3> matrix_to_euler(const Eigen::Matrix<Skalar, 3, 3>& mat) {
  double pitch = -asin(mat(2, 0));
  double yaw, roll;
  // Catch Gimbal locks
  if ((1. - std::numeric_limits<double>::epsilon() <= mat(2, 0))
      and (mat(2, 0) <= 1. + std::numeric_limits<double>::epsilon())) {
    yaw = 0;
    roll = atan2(-mat(0, 1), -mat(0, 2));
  } else if ((-1. - std::numeric_limits<double>::epsilon())
             and (mat(2, 0) <= -1. + std::numeric_limits<double>::epsilon())) {
    yaw = 0;
    roll = atan2(mat(0, 1), mat(0, 2));
  } else {
    yaw = atan2(mat(1, 0), mat(0, 0));
    roll = atan2(mat(2, 1), mat(2, 2));
  }
  return {roll, pitch, yaw};
}

template <typename Skalar>
inline Eigen::Vector<Skalar, 3> quaternion_to_euler(const Eigen::Quaternion<Skalar>& quat) {
  auto ret = matrix_to_euler(quat.toRotationMatrix());
  // auto ret = quaternion_to_euler(quat, 2, 1, 0, false);
  if (std::isnan(ret(0)) or std::isnan(ret(1)) or std::isnan(ret(2))) {
    std::cout << "!!!!!!!!!!!!NAN detected in quaternion_to_euler: " << quat << ": " << ret << std::endl;
  }
  return {ret(0), ret(1), ret(2)};
}

template <typename Skalar>
inline Skalar yaw_from_quaternion(const Eigen::Quaternion<Skalar>& quat) {
  Skalar yaw;
  Eigen::Matrix<Skalar, 3, 3> rotation_matrix = quat.toRotationMatrix();
  yaw = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
  return yaw;
}

template <typename Skalar>
inline Eigen::Quaternion<Skalar> yaw_to_quaternion(Skalar yaw) {
  Eigen::Quaternion<Skalar> q(Eigen::AngleAxis<Skalar>(yaw, Eigen::Vector<Skalar, 3>::UnitZ()));
  return q;
}

template <typename Skalar>
inline Eigen::Quaternion<Skalar> yaw_quaternion_from_quaternion(const Eigen::Quaternion<Skalar>& quat) {
  Eigen::Quaternion<Skalar> q(Eigen::AngleAxis<Skalar>(yaw_from_quaternion(quat), Eigen::Vector<Skalar, 3>::UnitZ()));
  return q;
}

template <typename Skalar>
inline Eigen::Matrix<Skalar, 3, 3> skew_matrix(const Eigen::Vector<Skalar, 3>& point) {
  Eigen::Matrix<Skalar, 3, 3> skew_matrix;
  skew_matrix << 0, -point.z(), point.y(), point.z(), 0, -point.x(), -point.y(), point.x(), 0;
  return skew_matrix;
}

template <typename Skalar>
inline Eigen::Matrix<Skalar, 4, 4> get_transformation_matrix(const Eigen::Quaternion<Skalar>& orientation,
                                                             const Eigen::Vector<Skalar, 3>& point) {
  Eigen::Matrix<Skalar, 4, 4> m;
  m.template block<3, 3>(0, 0) = orientation.toRotationMatrix();
  m.template block<3, 1>(3, 0) = point;

  m(3, 3) = 1;
  return m;
};
