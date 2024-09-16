
#ifndef FOOTSWINGTRAJECTORY_H
#define FOOTSWINGTRAJECTORY_H

#include <vector>
// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

#include "common/Interpolate.hpp"

using Vec3 = typename Eigen::Matrix<double, 3, 1>;

class FootSwingTrajectory {
 private:
  Vec3 _p0, _pf, _p, _v, _a;
  double _height;
  double _world_blend;

 public:
  FootSwingTrajectory() {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
    _world_blend = 1.0;
  }
  void setInitialPosition(Vec3 p0) { _p0 = p0; }

  void setFinalPosition(Vec3 pf) { _pf = pf; }

  void setWorldBlend(double world_blend) { _world_blend = world_blend; }

  const Eigen::Vector3d& getInitialPosition() const { return _p0; }

  const Eigen::Vector3d& getFinalPosition() const { return _pf; }

  void setHeight(double h) { _height = h; }

  void computeSwingTrajectoryBezier(double phase, double swingTime);

  Vec3 getPosition() { return _p; }

  Vec3 getVelocity() { return _v; }

  Vec3 getAcceleration() { return _a; }
};

#endif  // FOOTSWINGTRAJECTORY_H