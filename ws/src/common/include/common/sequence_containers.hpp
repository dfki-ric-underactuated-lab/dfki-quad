#pragma once
#include <Eigen/Core>
#include <array>
#include <vector>

template <typename T, std::size_t len>
class SequenceView {
 private:
  T* data_;

 public:
  constexpr T& operator[](unsigned int idx) const { return data_[idx]; }
  constexpr int size() const noexcept { return len; }

  // Constructors
  SequenceView(T* data) : data_(data) {}
  SequenceView(std::array<T, len>& array) : data_(array.data()) {}
  SequenceView(std::vector<T>& vector) : data_(vector.data()) { assert(vector.size() == len); }
};

//// For bool vectors everything is different
// template <std::size_t len>
// class SequenceView<bool, len> {
//  private:
//   std::vector<bool>& vec_;
//
//  public:
//   std::vector<bool>::reference operator[](unsigned int idx) const { return vec_[idx]; }
//   constexpr int size() const noexcept { return len; }
//
//   // Constructors
//   SequenceView(std::vector<bool>& vector) : vec_(vector) { assert(vector.size() == len); }
// };

template <std::size_t len, typename T>
Eigen::Map<const Eigen::Matrix<double, len, 1>> as_eigen_vector(const std::vector<T>& vector) {
  assert(vector.size() == len);
  return Eigen::Map<const Eigen::Matrix<double, len, 1>>(vector.data());
}

template <std::size_t len, typename T>
std::array<T, len> to_array(const std::vector<T>& vector) {
  assert(vector.size() == len);
  std::array<T, len> arr;
  for (unsigned int idx = 0; idx < len; idx++) {
    arr[idx] = vector[idx];
  }
  return arr;
}

template <std::size_t len, typename T>
std::vector<T> to_vector(const std::array<T, len>& arr) {
  std::vector<T> vector(len);
  for (unsigned int idx = 0; idx < len; idx++) {
    vector[idx] = arr[idx];
  }
  return vector;
}

template <int len, typename T>
std::vector<T> to_vector(const Eigen::Matrix<T, len, 1>& vec) {
  std::vector<T> vector(len);
  for (unsigned int idx = 0; idx < len; idx++) {
    vector[idx] = vec[idx];
  }
  return vector;
}

template <typename T, int lenA, int lenB>
Eigen::Matrix<T, lenA + lenB, 1> stack(const Eigen::Matrix<T, lenA, 1>& vecA, const Eigen::Matrix<T, lenB, 1>& vecB) {
  Eigen::Matrix<T, lenA + lenB, 1> vec;
  vec << vecA, vecB;
  return vec;
}

template <typename T, std::size_t total_len, std::size_t outer_len, typename innerT>
void assign(const std::array<innerT, outer_len>& from, std::array<T, total_len>& to) {
  static_assert(total_len % outer_len == 0);
  std::size_t inner_len = total_len / outer_len;
  for (std::size_t outer_idx = 0; outer_idx < outer_len; outer_idx++) {
    assert(from[outer_idx].size() == inner_len);
    for (std::size_t inner_idx = 0; inner_idx < inner_len; inner_idx++) {
      to[outer_idx * inner_len + inner_idx] = from[outer_idx][inner_idx];
    }
  }
}

template <std::size_t total_len, std::size_t outer_len, typename T, typename innerT>
void assign(const std::array<innerT, outer_len>& from, T* to) {
  static_assert(total_len % outer_len == 0);
  std::size_t inner_len = total_len / outer_len;
  for (std::size_t outer_idx = 0; outer_idx < outer_len; outer_idx++) {
    assert(from[outer_idx].size() == inner_len);
    for (std::size_t inner_idx = 0; inner_idx < inner_len; inner_idx++) {
      *(to + inner_idx) = from[outer_idx][inner_idx];
    }
  }
}

template <std::size_t len>
std::array<bool, len> invert(const std::array<bool, len>& arr) {
  std::array<bool, len> out_arr;
  for (std::size_t i = 0; i < len; i++) {
    out_arr[i] = !arr[i];
  }
  return out_arr;
}