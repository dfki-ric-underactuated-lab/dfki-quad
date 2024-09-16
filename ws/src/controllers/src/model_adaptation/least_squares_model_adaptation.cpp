#include "model_adaptation/least_squares_model_adaptation.hpp"

#include "common/quaternion_operations.hpp"

using namespace std;

LeastSquaresModelAdaptation::LeastSquaresModelAdaptation(std::unique_ptr<ModelInterface> quad_model,
                                                         std::unique_ptr<StateInterface> initial_state,
                                                         int buffer_size,
                                                         Eigen::DiagonalMatrix<double, 10> Gamma,
                                                         double lambda)
    : state_(std::move(initial_state)), buffer_size_(buffer_size), output_filter_(5), Gamma_(Gamma), lambda_(lambda) {
  // (void)quad_model;     // TODO: Remove line when parameter is actually used,
  // this just prevents the compiler warning (void)initial_state;  // TODO:
  // Remove line when parameter is actually used, this just prevents the
  // compiler warning Initialize P-Matrix for now with identity

  P_.setIdentity();
  // initialize parameter vector
  Eigen::Matrix3d I_mat = quad_model->GetInertia();
  Eigen::Translation3d com = quad_model->GetBodyToCOM();
  double m = quad_model->GetMass();
  phi_ << m, com.x(), com.y(), com.z(), I_mat(0, 0), I_mat(0, 1), I_mat(0, 2), I_mat(1, 1), I_mat(1, 2), I_mat(2, 2);
  output_filter_.update(phi_);
  contact_force_buffer_ = state_->GetContactForces();
  for (int i = 0; i < ModelInterface::N_LEGS; i++) {
    foot_position_buffer_[i] =
        quad_model->GetFootPositionInWorld(i,
                                           state_->GetPositionInWorld(),
                                           state_->GetOrientationInWorld(),
                                           Eigen::Map<const Eigen::Vector3d>(state_->GetJointPositions()[i].data()));
  }
  orientation_buffer_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  linear_acc_buffer_ = state_->GetLinearAccInWorld();
  angular_acc_buffer_ = state_->GetAngularAccInWorld();
  msg_count_ = 1;
}

void LeastSquaresModelAdaptation::UpdateState(const StateInterface& state) { *state_ = state; }

bool LeastSquaresModelAdaptation::DoModelAdaptation(ModelInterface& model) {
  //(void)state_;  // TODO: Remove line when parameter is actually used, this
  // just prevents the compiler warning (void)model;  // TODO: Remove line when
  // parameter is actually used, this just prevents the compiler warning
  if (msg_count_ != buffer_size_) {
    // collect data as a batch
    std::array<Eigen::Vector3d, ModelInterface::N_LEGS> current_contact_forces = state_->GetContactForces();
    for (int i = 0; i < ModelInterface::N_LEGS; i++) {
      contact_force_buffer_[i] += current_contact_forces[i];
      foot_position_buffer_[i] += model.GetFootPositionInWorld(i, *state_);
    }
    orientation_buffer_ += quaternion_to_euler(state_->GetOrientationInWorld(), 3, 2, 1);
    linear_acc_buffer_ += state_->GetLinearAccInWorld();
    angular_acc_buffer_ += state_->GetAngularAccInWorld();
    msg_count_ += 1;

    return false;
  } else {
    // get averages for the batch
    std::array<Eigen::Vector3d, ModelInterface::N_LEGS> contact_forces;
    std::array<Eigen::Vector3d, ModelInterface::N_LEGS> foot_positions;
    for (int i = 0; i < ModelInterface::N_LEGS; i++) {
      contact_forces[i] = contact_force_buffer_[i] / buffer_size_;
      contact_force_buffer_[i].setZero();
      foot_positions[i] = foot_position_buffer_[i] / buffer_size_;
      foot_position_buffer_[i].setZero();
    }
    Eigen::Vector3d orientation = orientation_buffer_ / buffer_size_;
    orientation_buffer_.setZero();
    Eigen::Vector3d linear_acc = linear_acc_buffer_ / buffer_size_;
    linear_acc_buffer_.setZero();
    Eigen::Vector3d angular_acc = angular_acc_buffer_ / buffer_size_;
    angular_acc_buffer_.setZero();
    msg_count_ = 0;
    // compute regressor matrix
    Eigen::Matrix<double, 6, 10> y;
    RegressorMatrix(orientation, linear_acc, angular_acc, contact_forces, y);
    // get generalized forces
    Eigen::Vector<double, 6> b;
    bVec(contact_forces, foot_positions, b);
    // define some helper variables
    Eigen::Matrix<double, 6, 6> I, A;
    I.setIdentity();
    A = lambda_ * I + y * P_ * y.transpose();
    // update P-Matrix TODO: include forgetting factor, Enable resetting
    P_ = Gamma_
         * (1.0 / lambda_ * P_
            - 1.0 / lambda_ * P_ * y.transpose() * A.completeOrthogonalDecomposition().pseudoInverse() * y * P_);
    // update parameter vector TODO: include forgetting factor, Enable resetting
    Eigen::Vector<double, 10> delta = P_ * y.transpose() * (b - y * phi_);
    output_filter_.update(output_filter_.get() + delta);
    phi_ = output_filter_.get();
    // std::cout << "Phi: " << phi_.transpose() << std::endl;
    // std::cout << "P:\n" << P_ << std::endl;
    Eigen::Vector<bool, 10> is_confident = P_.diagonal().array() < 1;
    if (is_confident.any()) {
      for (int i = 0; i < 10; i++) {
        if (is_confident(i)) {
          switch (i) {
            case 0:
              model.SetMass(phi_(0));
              break;
            case 1:
              model.SetCOM(0, phi_(1));
              break;
            case 2:
              model.SetCOM(1, phi_(2));
              break;
            case 3:
              model.SetCOM(2, phi_(3));
              break;
            case 4:
              model.SetInertia(0, 0, phi_(4));
              break;
            case 5:
              model.SetInertia(0, 1, phi_(5));
              model.SetInertia(1, 0, phi_(5));
              break;
            case 6:
              model.SetInertia(0, 2, phi_(6));
              model.SetInertia(2, 0, phi_(6));
              break;
            case 7:
              model.SetInertia(1, 1, phi_(7));
              break;
            case 8:
              model.SetInertia(1, 2, phi_(8));
              model.SetInertia(2, 1, phi_(8));
              break;
            case 9:
              model.SetInertia(2, 2, phi_(9));
              break;
            default:
              break;
          }
        }
      }
      return true;
    } else {
      return false;
    }
  }
}

void LeastSquaresModelAdaptation::RegressorMatrix(const Eigen::Vector3d& orientation,
                                                  const Eigen::Vector3d& linear_acceleration,
                                                  const Eigen::Vector3d& angular_acceleration,
                                                  const std::array<Eigen::Vector3d, 4>& contact_forces,
                                                  Eigen::Matrix<double, 6, 10>& y) const {
  // save inputs into own variables for readability
  double phi_x = orientation(0);
  double phi_y = orientation(1);
  double phi_z = orientation(2);
  double pdd_x = linear_acceleration(0);
  double pdd_y = linear_acceleration(1);
  double pdd_z = linear_acceleration(2);
  double omegad_x = angular_acceleration(0);
  double omegad_y = angular_acceleration(1);
  double omegad_z = angular_acceleration(2);
  double ffl_x = contact_forces[0](0);
  double ffl_y = contact_forces[0](1);
  double ffl_z = contact_forces[0](2);
  double ffr_x = contact_forces[1](0);
  double ffr_y = contact_forces[1](1);
  double ffr_z = contact_forces[1](2);
  double fbl_x = contact_forces[2](0);
  double fbl_y = contact_forces[2](1);
  double fbl_z = contact_forces[2](2);
  double fbr_x = contact_forces[3](0);
  double fbr_y = contact_forces[3](1);
  double fbr_z = contact_forces[3](2);

  // create intermediate variables with often occuring values
  double t1 = std::cos(phi_x);
  double t2 = std::sin(phi_x);
  double t3 = std::cos(phi_y);
  double t4 = std::sin(phi_y);
  double t5 = std::cos(phi_z);
  double t6 = std::sin(phi_z);
  double t7 = t1 * t1;
  double t8 = t2 * t2;
  double t9 = t3 * t3;
  double t10 = t4 * t4;
  double t11 = t5 * t5;
  double t12 = t6 * t6;

  // populate regressor matrix
  y.setZero();
  y(0, 0) = pdd_x;
  y(1, 0) = pdd_y;
  y(2, 0) = g_ + pdd_z;
  y(3, 2) = -fbl_z - fbr_z - ffl_z - ffr_z;
  y(3, 3) = fbl_y + fbr_y + ffl_y + ffr_y;
  y(3, 4) = omegad_x * t9 * t11 + omegad_y * t6 * t9 * t5 - omegad_z * t4 * t3 * t5;
  y(3, 5) = omegad_z * (t5 * t2 * t9 + t4 * (t1 * t6 - t5 * t2 * t4))
            + omegad_y * (t3 * t5 * (t1 * t5 + t2 * t4 * t6) - t3 * t6 * (t1 * t6 - t5 * t2 * t4))
            - 2.0 * omegad_x * t3 * t5 * (t1 * t6 - t5 * t2 * t4);
  y(3, 6) = 2.0 * omegad_x * t3 * t5 * (t2 * t6 + t1 * t5 * t4)
            - omegad_y * (t3 * t5 * (t5 * t2 - t1 * t4 * t6) - t3 * t6 * (t2 * t6 + t1 * t5 * t4))
            - omegad_z * (t4 * (t2 * t6 + t1 * t5 * t4) - t1 * t9 * t5);
  y(3, 7) = omegad_x * (t1 * t6 - t5 * t2 * t4) * (t1 * t6 - t5 * t2 * t4)
            - omegad_y * (t1 * t5 + t2 * t4 * t6) * (t1 * t6 - t5 * t2 * t4)
            - omegad_z * t3 * t2 * (t1 * t6 - t5 * t2 * t4);
  y(3, 8) =
      omegad_y
          * ((t1 * t5 + t2 * t4 * t6) * (t2 * t6 + t1 * t5 * t4) + (t1 * t6 - t5 * t2 * t4) * (t5 * t2 - t1 * t4 * t6))
      - omegad_z * (t1 * t3 * (t1 * t6 - t5 * t2 * t4) - t3 * t2 * (t2 * t6 + t1 * t5 * t4))
      - 2.0 * omegad_x * (t2 * t6 + t1 * t5 * t4) * (t1 * t6 - t5 * t2 * t4);
  y(3, 9) = omegad_x * (t2 * t6 + t1 * t5 * t4) * (t2 * t6 + t1 * t5 * t4)
            - omegad_y * (t2 * t6 + t1 * t5 * t4) * (t5 * t2 - t1 * t4 * t6)
            + omegad_z * t1 * t3 * (t2 * t6 + t1 * t5 * t4);
  y(4, 1) = fbl_z + fbr_z + ffl_z + ffr_z;
  y(4, 3) = -fbl_x - fbr_x - ffl_x - ffr_x;
  y(4, 4) = omegad_y * t9 * t12 + omegad_x * t5 * t9 * t6 - omegad_z * t4 * t3 * t6;
  y(4, 5) = omegad_x * (t3 * t5 * (t1 * t5 + t2 * t4 * t6) - t3 * t6 * (t1 * t6 - t5 * t2 * t4))
            - omegad_z * (t4 * (t1 * t5 + t2 * t4 * t6) - t9 * t2 * t6)
            + 2.0 * omegad_y * t3 * t6 * (t1 * t5 + t2 * t4 * t6);
  y(4, 6) = omegad_z * (t1 * t6 * t9 + t4 * (t5 * t2 - t1 * t4 * t6))
            - omegad_x * (t3 * t5 * (t5 * t2 - t1 * t4 * t6) - t3 * t6 * (t2 * t6 + t1 * t5 * t4))
            - 2.0 * omegad_y * t3 * t6 * (t5 * t2 - t1 * t4 * t6);
  y(4, 7) = omegad_y * (t1 * t5 + t2 * t4 * t6) * (t1 * t5 + t2 * t4 * t6)
            - omegad_x * (t1 * t5 + t2 * t4 * t6) * (t1 * t6 - t5 * t2 * t4)
            + omegad_z * t3 * t2 * (t1 * t5 + t2 * t4 * t6);
  y(4, 8) = omegad_z * (t1 * t3 * (t1 * t5 + t2 * t4 * t6) - t3 * t2 * (t5 * t2 - t1 * t4 * t6))
            + omegad_x
                  * ((t1 * t5 + t2 * t4 * t6) * (t2 * t6 + t1 * t5 * t4)
                     + (t1 * t6 - t5 * t2 * t4) * (t5 * t2 - t1 * t4 * t6))
            - 2.0 * omegad_y * (t1 * t5 + t2 * t4 * t6) * (t5 * t2 - t1 * t4 * t6);
  y(4, 9) = omegad_y * (t5 * t2 - t1 * t4 * t6) * (t5 * t2 - t1 * t4 * t6)
            - omegad_x * (t2 * t6 + t1 * t5 * t4) * (t5 * t2 - t1 * t4 * t6)
            - omegad_z * t1 * t3 * (t5 * t2 - t1 * t4 * t6);
  y(5, 1) = -fbl_y - fbr_y - ffl_y - ffr_y;
  y(5, 2) = fbl_x + fbr_x + ffl_x + ffr_x;
  y(5, 4) = omegad_z * t10 - omegad_x * t3 * t5 * t4 - omegad_y * t3 * t4 * t6;
  y(5, 5) = omegad_x * (t5 * t2 * t9 + t4 * (t1 * t6 - t5 * t2 * t4))
            - omegad_y * (t4 * (t1 * t5 + t2 * t4 * t6) - t9 * t2 * t6) - 2.0 * omegad_z * t3 * t2 * t4;
  y(5, 6) = omegad_y * (t1 * t6 * t9 + t4 * (t5 * t2 - t1 * t4 * t6))
            - omegad_x * (t4 * (t2 * t6 + t1 * t5 * t4) - t1 * t9 * t5) - 2.0 * omegad_z * t1 * t3 * t4;
  y(5, 7) = omegad_z * t9 * t8 - omegad_x * t3 * t2 * (t1 * t6 - t5 * t2 * t4)
            + omegad_y * t3 * t2 * (t1 * t5 + t2 * t4 * t6);
  y(5, 8) = omegad_y * (t1 * t3 * (t1 * t5 + t2 * t4 * t6) - t3 * t2 * (t5 * t2 - t1 * t4 * t6))
            - omegad_x * (t1 * t3 * (t1 * t6 - t5 * t2 * t4) - t3 * t2 * (t2 * t6 + t1 * t5 * t4))
            + 2.0 * omegad_z * t1 * t9 * t2;
  y(5, 9) = omegad_z * t7 * t9 + omegad_x * t1 * t3 * (t2 * t6 + t1 * t5 * t4)
            - omegad_y * t1 * t3 * (t5 * t2 - t1 * t4 * t6);
}

void LeastSquaresModelAdaptation::bVec(const std::array<Eigen::Vector3d, 4>& contact_forces,
                                       const std::array<Eigen::Vector3d, 4>& foot_positions,
                                       Eigen::Vector<double, 6>& b) const {
  // sum of forces acting on torso
  Eigen::Vector3d forces = Eigen::Vector3d::Zero();
  Eigen::Vector3d torques = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; i++) {
    forces += contact_forces[i];
    torques += contact_forces[i].cross(foot_positions[i]);
  }
  b.head<3>() = forces;   // set first 3 elements to forces
  b.tail<3>() = torques;  // and last 3 to torques
}
