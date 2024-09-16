#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cassert>
#include <limits>
#include <numeric>
#include <type_traits>
#include <vector>

template <typename T, unsigned long N, typename numT>
std::array<T, N> operator/(const std::array<T, N>& container, numT divisor) {
  std::array<T, N> lhs(container);
  for (unsigned int idx = 0; idx < container.size(); idx++) {
    lhs[idx] = container[idx] / divisor;
  }
  return lhs;
}

template <typename T, unsigned long N>
std::array<T, N> operator+(const std::array<T, N>& summandA, const std::array<T, N>& summandB) {
  std::array<T, N> result;
  for (unsigned int idx = 0; idx < summandA.size(); idx++) {
    result[idx] = summandA[idx] + summandB[idx];
  }
  return result;
}

template <typename T, unsigned long N>
std::array<T, N> operator-(const std::array<T, N>& subrahendA, const std::array<T, N>& subrahendB) {
  std::array<T, N> result;
  for (unsigned int idx = 0; idx < subrahendA.size(); idx++) {
    result[idx] = subrahendA[idx] - subrahendB[idx];
  }
  return result;
}

template <typename T, unsigned long N>
std::array<T, N>& operator+=(std::array<T, N>& summandA, std::array<T, N>& summandB) {
  for (unsigned int idx = 0; idx < summandA.size(); idx++) {
    summandA[idx] = summandA[idx] + summandB[idx];
  }
  return summandA;
}

template <typename T, unsigned long N>
std::array<T, N>& operator-=(std::array<T, N>& subrahendA, std::array<T, N>& subrahendB) {
  for (unsigned int idx = 0; idx < subrahendA.size(); idx++) {
    subrahendA[idx] = subrahendA[idx] - subrahendB[idx];
  }
  return subrahendA;
}

template <typename T, typename numT>
std::vector<T> operator/(const std::vector<T>& container, numT divisor) {
  std::vector<T> lhs(container);
  for (unsigned int idx = 0; idx < container.size(); idx++) {
    lhs[idx] = container[idx] / divisor;
  }
  return lhs;
}

template <typename T>
std::vector<T> operator+(const std::vector<T>& summandA, const std::vector<T>& summandB) {
  assert(summandA.size() == summandB.size());
  std::vector<T> result;
  result.resize(summandA.size());
  for (unsigned int idx = 0; idx < summandA.size(); idx++) {
    result[idx] = summandA[idx] + summandB[idx];
  }
  return result;
}

template <typename T>
std::vector<T> operator-(const std::vector<T>& subrahendA, const std::vector<T>& subrahendB) {
  assert(subrahendA.size() == subrahendB.size());
  std::vector<T> result;
  result.resize(subrahendA.size());

  for (unsigned int idx = 0; idx < subrahendA.size(); idx++) {
    result[idx] = subrahendA[idx] - subrahendB[idx];
  }
  return result;
}

template <typename T>
std::vector<T>& operator+=(std::vector<T>& summandA, std::vector<T>& summandB) {
  assert(summandA.size() == summandB.size());
  for (unsigned int idx = 0; idx < summandA.size(); idx++) {
    summandA[idx] = summandA[idx] + summandB[idx];
  }
  return summandA;
}

template <typename T>
std::vector<T>& operator-=(std::vector<T>& subrahendA, std::vector<T>& subrahendB) {
  assert(subrahendA.size() == subrahendB.size());
  for (unsigned int idx = 0; idx < subrahendA.size(); idx++) {
    subrahendA[idx] = subrahendA[idx] - subrahendB[idx];
  }
  return subrahendA;
}

template <typename T>
class MovingAverage {
 private:
  unsigned int buffersize_;
  unsigned int index_ = 0;
  std::vector<T> buffer_;
  T val_;

 public:
  MovingAverage(unsigned int buffersize = 1) : buffersize_(buffersize), val_(T{}) { buffer_.reserve(buffersize); };

  T update(T value) {
    if (buffer_.size() == buffersize_) {
      val_ -= buffer_[index_];
      buffer_[index_++] = value;
      val_ += value;
      index_ %= buffersize_;
      return val_ / buffersize_;
    } else if (buffer_.size() > 0) {
      buffer_.push_back(value);
      val_ += value;
      return val_ / buffer_.size();
    } else {
      buffer_.push_back(value);
      val_ = value;  // If there is a vector in it addition wont work due to sizes, thats why extra case
      return val_;
    }
  }

  T get() const { return val_ / buffer_.size(); }
  void resize(unsigned int size) {
    if (size == buffersize_) {
      return;
    } else if (size == buffer_.size()) {
      buffersize_ = size;
      return;
    } else if (size > buffer_.size()) {
      buffer_.reserve(size);
      std::rotate(buffer_.begin(), buffer_.begin() + index_, buffer_.end());
      index_ = 0;
      buffersize_ = size;
    } else {
      std::rotate(buffer_.begin(), buffer_.begin() + ((index_ - size) % buffer_.size()), buffer_.end());
      buffer_.resize(size);
      index_ = 0;
      val_ = std::accumulate(buffer_.begin(), buffer_.end(), T{});
    }
  }
};

template <>
class MovingAverage<Eigen::Quaterniond> {
 private:
  unsigned int buffersize_;
  unsigned int index_ = 0;
  std::vector<Eigen::Quaterniond> buffer_;
  Eigen::Matrix4d val_;
  Eigen::Quaterniond largest_eigenvector_;

 public:
  MovingAverage(unsigned int buffersize = 1) : buffersize_(buffersize), val_(Eigen::Matrix4d::Zero()) {
    buffer_.reserve(buffersize);
  };

  Eigen::Quaterniond update(Eigen::Quaterniond value) {
    if (buffer_.size() == buffersize_) {
      val_ -= buffer_[index_].coeffs() * buffer_[index_].coeffs().transpose();
      buffer_[index_++] = value;
      val_ += value.coeffs() * value.coeffs().transpose();
      index_ %= buffersize_;
    } else if (buffer_.size() > 0) {
      buffer_.push_back(value);
      val_ += value.coeffs() * value.coeffs().transpose();
    } else {
      buffer_.push_back(value);
      val_ = value.coeffs() * value.coeffs().transpose();
      // If there is a vector in it addition wont work due to sizes, thats why extra case
    }
    largest_eigenvector_ = Eigen::JacobiSVD(val_, Eigen::ComputeFullU).matrixU().col(0);
    return largest_eigenvector_;
  }

  Eigen::Quaterniond get() const { return largest_eigenvector_; }
};

template <typename T>  // TODO: improve algorithm
class MovingMin {
 private:
  unsigned int buffersize_;
  unsigned int index_ = 0;
  std::vector<T> buffer_;
  T val_;
  unsigned int min_idx_ = 0;

 public:
  MovingMin(unsigned int buffersize = 1) : buffersize_(buffersize), val_(std::numeric_limits<T>::max()) {
    buffer_.reserve(buffersize);
  };

  T update(T value) {
    if (buffer_.size() == buffersize_) {
      if (value < val_) {
        val_ = value;
        min_idx_ = index_;
      } else if (min_idx_ == index_) {
        buffer_[index_++] = value;
        auto min_iter = std::min_element(buffer_.begin(), buffer_.end());
        min_idx_ = std::distance(buffer_.begin(), min_iter);
        val_ = *min_iter;
        index_ %= buffersize_;
        return val_;
      }
      buffer_[index_++] = value;
      index_ %= buffersize_;
      return val_;
    } else if (buffer_.size() > 0) {
      buffer_.push_back(value);
      if (value < val_) {
        val_ = value;
        min_idx_ = buffer_.size() - 1;
      }
      return val_;
    } else {
      buffer_.push_back(value);
      val_ = value;  // If there is a vector in it addition wont work due to sizes, thats why extra case
      return val_;
    }
  }

  T get() const { return val_; }

  void resize(unsigned int size) {
    if (size == buffersize_) {
      return;
    } else if (size == buffer_.size()) {
      buffersize_ = size;
      return;
    } else if (size > buffer_.size()) {
      buffer_.reserve(size);
      std::rotate(buffer_.begin(), buffer_.begin() + index_, buffer_.end());
      index_ = 0;
      buffersize_ = size;
    } else {
      std::rotate(buffer_.begin(), buffer_.begin() + ((index_ - size) % buffer_.size()), buffer_.end());
      buffer_.resize(size);
      index_ = 0;
      auto min_iter = std::min_element(buffer_.begin(), buffer_.end());
      min_idx_ = std::distance(buffer_.begin(), min_iter);
      val_ = buffer_[min_idx_];
    }
  }
};

template <typename T>  // TODO: improve algorithm
class MovingMax {
 private:
  unsigned int buffersize_;
  unsigned int index_ = 0;
  std::vector<T> buffer_;
  T val_;
  unsigned int max_idx_ = 0;

 public:
  MovingMax(unsigned int buffersize = 1) : buffersize_(buffersize), val_(std::numeric_limits<T>::min()) {
    buffer_.reserve(buffersize);
  };

  T update(T value) {
    if (buffer_.size() == buffersize_) {
      if (value > val_) {
        val_ = value;
        max_idx_ = index_;
      } else if (max_idx_ == index_) {
        buffer_[index_++] = value;
        auto max_iter = std::max_element(buffer_.begin(), buffer_.end());
        max_idx_ = std::distance(buffer_.begin(), max_iter);
        val_ = *max_iter;
        index_ %= buffersize_;
        return val_;
      }
      buffer_[index_++] = value;
      index_ %= buffersize_;
      return val_;
    } else if (buffer_.size() > 0) {
      buffer_.push_back(value);
      if (value > val_) {
        val_ = value;
        max_idx_ = buffer_.size() - 1;
      }
      return val_;
    } else {
      buffer_.push_back(value);
      val_ = value;  // If there is a vector in it addition wont work due to sizes, thats why extra case
      return val_;
    }
  }

  T get() const { return val_; }

  void resize(unsigned int size) {
    if (size == buffersize_) {
      return;
    } else if (size == buffer_.size()) {
      buffersize_ = size;
      return;
    } else if (size > buffer_.size()) {
      buffer_.reserve(size);
      std::rotate(buffer_.begin(), buffer_.begin() + index_, buffer_.end());
      index_ = 0;
      buffersize_ = size;
    } else {
      std::rotate(buffer_.begin(), buffer_.begin() + ((index_ - size) % buffer_.size()), buffer_.end());
      buffer_.resize(size);
      index_ = 0;
      auto max_iter = std::max_element(buffer_.begin(), buffer_.end());
      max_idx_ = std::distance(buffer_.begin(), max_iter);
      val_ = buffer_[max_idx_];
    }
  }
};

template <typename T, unsigned int SIZE>
class MovingAverage2 {
 private:
  bool full = false;
  unsigned int index_ = 0;
  std::array<T, SIZE> buffer_;
  T val_;

 public:
  MovingAverage2() : buffer_(std::array<T, SIZE>{}), val_(T{}){};

  T update(T value) {
    if (full) {
      val_ -= buffer_[index_];
      buffer_[index_++] = value;
      val_ += value;
      index_ %= SIZE;
      return val_ / SIZE;
    } else {
      buffer_[index_++];
      val_ += value;
      if (index_ == SIZE) {
        full = true;
        index_ = 0;
        return val_ / SIZE;
      }
      return val_ / index_;
    }
  }

  T get() {
    if (full) {
      return val_ / SIZE;
    } else {
      return val_ / index_;
    }
  }
};
