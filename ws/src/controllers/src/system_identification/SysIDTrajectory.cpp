#include "system_identification/SysIDTrajectory.hpp"

SysIdTrajectory::SysIdTrajectory(const Eigen::Ref<const Eigen::Vector3d> position,
                                 const double a,
                                 const double b,
                                 const double c)
    : position_(position), a_(a), b_(b), c_(c) {
  // segment parameters
  p0_.setZero();        // start point, spherical coordinates
  pf_.setZero();        // end point, spherical coordinates
  segment_time_ = 0.0;  // segment total time
  // current EE position and velocity, cartesian coordinates
  pos_.setZero();
  vel_.setZero();
}

void SysIdTrajectory::setInitialPosition(const Eigen::Ref<const Eigen::Vector2d> p0) { p0_ = p0; }

void SysIdTrajectory::setFinalPosition(const Eigen::Ref<const Eigen::Vector2d> pf) { pf_ = pf; }

void SysIdTrajectory::setEllipsoidParameters(const Eigen::Ref<const Eigen::Vector3d> position,
                                             const double a,
                                             const double b,
                                             const double c) {
  position_ = position;
  a_ = a;
  b_ = b;
  c_ = c;
}

void SysIdTrajectory::setSegmentTime(const double segment_time) { segment_time_ = segment_time; }

std::array<double, 3> SysIdTrajectory::getEllipsoidParams() const {
  std::array<double, 3> params = {a_, b_, c_};
  return params;
}

Eigen::Vector3d SysIdTrajectory::getEllipsoidPosition() const { return position_; }

Eigen::Vector3d SysIdTrajectory::getInitialEEPositionCartesian() const {
  return SphericalPos2Cartesian(p0_) + position_;
}

Eigen::Vector3d SysIdTrajectory::getFinalEEPositionCartesian() const { return SphericalPos2Cartesian(pf_) + position_; }

Eigen::Vector3d SysIdTrajectory::getCurrentEEPositionCartesian() const { return pos_; }

Eigen::Vector3d SysIdTrajectory::getCurrentEEVelocityCartesian() const { return vel_; }

double SysIdTrajectory::getSegmentTime() const { return segment_time_; }

void SysIdTrajectory::computeEllipsoidalTrajectory(const double timer) {
  Eigen::Vector2d pos, vel;
  double phase = timer / segment_time_;
  pos = Interpolate::cubicBezier<Eigen::Vector2d>(p0_, pf_, phase);
  pos_ = SphericalPos2Cartesian(pos) + position_;
  vel = Interpolate::cubicBezierFirstDerivative<Eigen::Vector2d>(p0_, pf_, phase);
  vel_ = SphericalVel2Cartesian(pos, vel);
}

Eigen::Vector3d SysIdTrajectory::SphericalPos2Cartesian(const Eigen::Ref<const Eigen::Vector2d> spherical) const {
  Eigen::Vector3d cartesian;
  // The ellipsoid shape is defined with semi-axis lengths a, b and c.
  // such that,
  //
  // x = a * sin(theta) * cos(phi)
  // y = b * sin(theta) * sin(phi)
  // z = c * cos(theta)
  //
  // where theta = [0,pi] and phi = [0,2*pi]
  // i.e. theta [spherical(0)] is the polar and phi [spherical(1)] is the azimuthal angle
  cartesian.x() = a_ * std::sin(spherical(0)) * std::cos(spherical(1));
  cartesian.y() = b_ * std::sin(spherical(0)) * std::sin(spherical(1));
  cartesian.z() = c_ * std::cos(spherical(0));

  return cartesian;
}

Eigen::Vector3d SysIdTrajectory::SphericalVel2Cartesian(const Eigen::Ref<const Eigen::Vector2d> spherical_pos,
                                                        const Eigen::Ref<const Eigen::Vector2d> spherical_vel) const {
  Eigen::Vector3d cartesian;
  cartesian.x() = a_
                  * (std::cos(spherical_pos(0)) * std::cos(spherical_pos(1)) * spherical_vel(0)
                     - std::sin(spherical_pos(0)) * std::sin(spherical_pos(1)) * spherical_vel(1));
  cartesian.y() = b_
                  * (std::cos(spherical_pos(0)) * std::sin(spherical_pos(1)) * spherical_vel(0)
                     + std::sin(spherical_pos(0)) * std::cos(spherical_pos(1)) * spherical_vel(1));
  cartesian.z() = -c_ * std::sin(spherical_pos(0)) * spherical_vel(0);

  return cartesian;
}

void SysIdTrajectory::sampleNextEEPosition() {
  // set start point to last end point
  p0_ = pf_;
  // providing a seed value
  std::random_device rd;
  std::mt19937 gen(rd());
  // number generators
  std::uniform_real_distribution<double> r1(0.0, PI);
  std::uniform_real_distribution<double> r2(0.0, 2.0 * PI);

  // polar angle between 0 and pi
  double theta = r1(gen);

  // azimuthal angle between 0 and 2*pi
  double phi = r2(gen);

  // assigning new spherical coordinates to the end point
  pf_ << theta, phi;

  std::cout << "pf_ = " << pf_.transpose() << std::endl;
}