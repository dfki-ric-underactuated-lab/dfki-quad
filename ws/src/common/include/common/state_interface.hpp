#pragma once
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class StateInterface {
 protected:
  StateInterface() = default;

 public:
  static const int NUM_FEET = 4;
  static const int NUM_JOINT_PER_FOOT = 3;
  using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::duration<long, std::nano>>;

  virtual const Eigen::Vector3d &GetPositionInWorld() const = 0;
  virtual const Eigen::Quaterniond &GetOrientationInWorld() const = 0;
  virtual const Eigen::Vector3d &GetLinearVelInWorld() const = 0;
  virtual const Eigen::Vector3d &GetAngularVelInWorld() const = 0;
  virtual const Eigen::Vector3d &GetLinearAccInWorld() const = 0;
  virtual const Eigen::Vector3d &GetAngularAccInWorld() const = 0;
  virtual const TimePoint &GetTime() const = 0;
  virtual const std::array<bool, NUM_FEET> &GetFeetContacts() const = 0;
  virtual const std::array<Eigen::Vector3d, NUM_FEET> &GetContactForces() const = 0;
  virtual const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> &GetJointPositions() const = 0;
  virtual const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> &GetJointVelocities() const = 0;
  virtual const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> &GetJointAccelerations() const = 0;
  virtual const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> &GetJointTorques() const = 0;

  virtual ~StateInterface(){};
  virtual StateInterface &operator=(const StateInterface &other) {
    (void)other;
    std::runtime_error("the assignment operator of StateInterface must always be overridden by deriving classes");
    return *this;
  };
};