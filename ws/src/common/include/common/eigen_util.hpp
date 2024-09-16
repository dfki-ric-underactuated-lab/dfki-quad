#pragma once
#include <Eigen/Dense>

/// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
template <class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived>& vec) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0,
          -vec[2],
          vec[1],
          vec[2],
          0.0,
          -vec[0],
          -vec[1],
          vec[0],
          0.0)
      .finished();
}