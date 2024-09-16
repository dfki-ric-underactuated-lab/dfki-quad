#include "quad_linear_matrices.hpp"

#include <Eigen/src/Core/Matrix.h>

Eigen::Matrix3d screw_symmetrical(Eigen::Vector3d v) {
  Eigen::Matrix3d sc;
  sc << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return sc;
};

// continuous time matrix
void get_quad_A_matrix(Eigen::Matrix3d& R_wb, Eigen::Matrix<double, 13, 13>& A) {
  A.setZero();
  A.block(0, 6, 3, 3) = R_wb;
  A.block(3, 9, 3, 3).setIdentity();
  A(11, 12) = -1.0;  // gravity state affects dotvz
};

// discrete time matrix
void get_quad_A_matrix(Eigen::Matrix3d& R_wb, double delta, Eigen::Matrix<double, 13, 13>& A) {
  get_quad_A_matrix(R_wb, A);  // get continuous A matrix
  A *= delta;
  A += Eigen::Matrix<double, 13, 13>::Identity();
};

// B matrix with full R
void get_quad_B_matrix(Eigen::Matrix3d& R_wb,
                       double m,
                       CentroidalTrajData& leg_data,
                       std::map<std::string, int>& u_index_map,
                       Eigen::Matrix<double, 3, 3>& I,
                       Eigen::Matrix<double, 13, 12>& B,
                       bool no_rotation = false) {
  B.setZero();
  // if "no_rotation" is set, ignore the inertia related rows.
  Eigen::Matrix3d I_rot;
  if (!no_rotation) {
    I_rot = R_wb * I * R_wb.transpose();  // rotate the inertia tensor
  }

  auto one_over_m_diag = Eigen::Matrix3d().Identity() / m;  // one over mass

  for (auto& leg : {"fl", "fr", "bl", "br"}) {
    // this seems to be not there in the original implementation
    // if(leg_data.contacts[leg].in_contact){

    // index for the matching column in B
    int start_col = u_index_map[leg];

    if (!no_rotation) {
      // find the position of the foot in the world frame
      Eigen::Vector3d foot_pos = leg_data.contacts[leg].base_to_foot_vect_in_world_frame;

      B.block(6, start_col, 3, 3) = I_rot.inverse() * screw_symmetrical(foot_pos);
    }

    B.block(9, start_col, 3, 3) = one_over_m_diag;
    //}
  }
};

// discrete time
void get_quad_B_matrix(Eigen::Matrix3d& R_wb,
                       double m,
                       CentroidalTrajData& leg_data,
                       std::map<std::string, int>& u_index_map,
                       Eigen::Matrix<double, 3, 3>& I,
                       double delta,
                       Eigen::Matrix<double, 13, 12>& B,
                       bool no_rotation = false) {
  get_quad_B_matrix(R_wb, m, leg_data, u_index_map, I, B, no_rotation);
  B *= delta;
};

// this assumes, that B is constant over the prediction horizon
// our understanding is that this is how it was done for mini cheetah.
// TODO: incorporate time varying B
void lift_AB(int N,
             const Eigen::MatrixXd& A_d,
             const Eigen::MatrixXd& B_d,
             Eigen::MatrixXd& A_lifted,
             Eigen::MatrixXd& B_lifted) {
  int n = A_d.rows();  // same as len(x)
  int m = B_d.cols();  // len(u)

  A_lifted.setZero();
  B_lifted.setZero();

  // first block of A_lifted is initialized with Identity matrix
  A_lifted.block(0, 0, n, n).setIdentity();

  // loop through rows and cols and populate lifted matrices (row refers to
  // block rows)
  for (int row = 1; row < N + 1; row++) {
    // block by block, take the last block and multiply with A_d
    // e.g. in row 1 the power is one
    Eigen::MatrixXd last_row_A_d = A_lifted.block((row - 1) * n, 0, n, n);
    A_lifted.block(row * n, 0, n, n) = last_row_A_d * A_d;
    for (int col = 0; col < N; col++) {
      int pow = row - col - 1;

      if (pow > 0) {
        // choose the correct block of A_lifted to get A**pow, which
        // goes into the B_lifted block
        Eigen::MatrixXd A_pow = A_lifted.block(n * pow, 0, n, n);
        B_lifted.block(row * (n), col * m, n, m) = A_pow * B_d;
      }
      // if blocks are on the main diagonal, set them to B_d
      else if (pow == 0) {
        B_lifted.block(row * (n), col * m, n, m) = B_d;
      }
    }
  }
}

// lift the Q matrix (block diagonal structure)
void lift_Q(int N, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Qf, Eigen::MatrixXd& Q_lifted) {
  int n = Q.rows();
  Q_lifted.setZero();
  for (int row = 0; row < N + 1; row++) {
    for (int col = 0; col < N + 1; col++) {
      if (row == col) {
        Q_lifted.block(row * n, col * n, n, n) = Q;
      }
    }
  }
  Q_lifted.block(N * n, N * n, n, n) = Qf;
}

// lift the R matrix (also block diagonal)
void lift_R(int N, const Eigen::MatrixXd& R, Eigen::MatrixXd& R_lifted) {
  int m = R.rows();
  R_lifted.setZero();
  for (int row = 0; row < N; row++) {
    for (int col = 0; col < N; col++) {
      if (row == col) {
        R_lifted.block(row * m, col * m, m, m) = R;
      }
    }
  }
}

// Ineq Constraint Matrix
// TODO: combine functions and remove magic numbers
// TODO: define lift Ce and lift ube for time variatn case
void lift_Ce(int N, const Eigen::MatrixXd& Ce, Eigen::MatrixXd& C_lifted) {
  int Ce_height = Ce.rows();  // 6
  int Ce_width = Ce.cols();   // 3
  C_lifted.setZero();
  for (int Ce_row = 0; Ce_row < N * 4; Ce_row++) {
    for (int Ce_col = 0; Ce_col < N * 4; Ce_col++) {
      if (Ce_col == Ce_row) {
        C_lifted.block(Ce_row * Ce_height, Ce_row * Ce_width, Ce_height, Ce_width) = Ce;
      }
    }
  }
}

// lift upper bound for constraint matrix
// TODO: also define for time variant case, by putting the ub_block assembly
// block into the second loop and expecting to get a vector of MPCLegData
// structs
// void lift_ube(int N, std::map<std::string, MPCLegData>& leg_data,
// Eigen::VectorXd& ub)
//{
//	// assemble one segment of ub (corresponding to one time step
//	Eigen::Vector<double, 4 * 6> ub_block;
//	int startidx = 0;
//	for (auto& leg : {"fl", "fr", "bl", "br"})
//	{
//		ub_block.segment(startidx, 6) << leg_data[leg].ube;
//		startidx += 6;
//	}
//
//	for (int row = 0; row < N; row++)
//	{
//		ub.segment(row * 4 * 6, 4 * 6) = ub_block;
//	}
//}

void lift_ube(int N, std::vector<CentroidalTrajData>& traj, Eigen::VectorXd& ub) {
  Eigen::Vector<double, N_FEET * N_CONSTRAINTS_PER_FEET> ub_block;
  int startidx;
  double fzmin, fzmax;
  for (int k = 0; k < N; k++) {
    startidx = 0;
    for (auto& leg : {"fl", "fr", "bl", "br"}) {
      fzmin = 0.0;
      fzmax = 0.0;
      if (traj[k].contacts[leg].in_contact) {
        fzmin = traj[k].contacts[leg].fzmin;
        fzmax = traj[k].contacts[leg].fzmax;
      }
      ub_block.segment(startidx, N_CONSTRAINTS_PER_FEET) << SMALL_NUMBER, SMALL_NUMBER, -fzmin, SMALL_NUMBER,
          SMALL_NUMBER, fzmax;
      startidx += N_CONSTRAINTS_PER_FEET;
    }
    ub.segment(k * N_FEET * N_CONSTRAINTS_PER_FEET, N_FEET * N_CONSTRAINTS_PER_FEET) = ub_block;
  }
}

// get empty trajectory to initialize the mpc with
// currently all fields are set to values that should
// make the quad stand, when no other node publishes a
// trajectory TODO: pass default xyz in header
void get_empty_trajectory(int N, std::vector<CentroidalTrajData>& traj) {
  traj.clear();
  Eigen::Vector<double, 12> default_xk;
  default_xk << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  CentroidalTrajData default_traj_data;
  ContactData default_contact_data;
  default_contact_data.in_contact = true;
  default_contact_data.fzmax = 100.0;
  default_contact_data.fzmin = 0.0;
  default_contact_data.mu = 0.5;

  // set to common default values...
  default_traj_data.xk = default_xk;
  default_traj_data.contacts["fl"] = default_contact_data;
  default_traj_data.contacts["fr"] = default_contact_data;
  default_traj_data.contacts["bl"] = default_contact_data;
  default_traj_data.contacts["br"] = default_contact_data;

  // ... only override the feet positions TODO: we could optionally set
  // these to the current values
  double default_x = 0.2;  // TODO find out good defaults and set them
                           // from config or simulation
  double default_y = 0.2;
  double default_z = -0.2;

  default_traj_data.contacts["fl"].foot_pos_in_body_frame << +default_x, +default_y, default_z;
  default_traj_data.contacts["fr"].foot_pos_in_body_frame << +default_x, -default_y, default_z;
  default_traj_data.contacts["bl"].foot_pos_in_body_frame << -default_x, +default_y, default_z;
  default_traj_data.contacts["br"].foot_pos_in_body_frame << -default_x, -default_y, default_z;

  default_traj_data.contacts["fl"].base_to_foot_vect_in_world_frame << +default_x, +default_y, default_z;
  default_traj_data.contacts["fr"].base_to_foot_vect_in_world_frame << +default_x, -default_y, default_z;
  default_traj_data.contacts["bl"].base_to_foot_vect_in_world_frame << -default_x, +default_y, default_z;
  default_traj_data.contacts["br"].base_to_foot_vect_in_world_frame << -default_x, -default_y, default_z;

  for (int i = 0; i < N; i++) {
    traj.push_back(default_traj_data);
  }
};
