#include <matplot/matplot.h>

#include <chrono>

#include "adaptive_gait_sequencer.hpp"
#include "bio_gait_sequencer.hpp"
#include "gait.hpp"
#include "gait_sequence.hpp"
#include "mpc.hpp"
#include "potato_sim/potato_sim.hpp"
#include "simple_gait_sequencer.hpp"
#include "swing_leg_controller.hpp"
#include "wrench_sequence.hpp"

#define BUFFER_SIZE 2000

int main() {
  BrickState quad_state;
  // quad_state.position_.z() = 0.17352385729468067;
  quad_state.position_.z() = 0.17352385729468067;
  quad_state.orientation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  quad_state.feet_contacts_ = {true, true, true, true};
  quad_state.virt_feet_positions_ = {Eigen::Vector3d(0.167, 0.1538, 0.0),
                                     Eigen::Vector3d(0.167, -0.1538, 0.0),
                                     Eigen::Vector3d(-0.197, 0.1538, 0.0),
                                     Eigen::Vector3d(-0.197, -0.1538, 0.0)};
  PotatoSimulator sim(quad_state);
  auto quad_model = sim.GetModel();

  // Swing Leg Controller
  SwingLegController sc(
      0.05, 1.0, 1.0, std::make_unique<BrickModel>(quad_model), std::make_unique<BrickState>(quad_state));

  // MPC
  double alpha = 4.e-5;
  Eigen::Matrix<double, 12, 1> state_weight{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.3, 0.3, 0.3, 0.2, 0.2, 0.1};
  double mu = 0.45;
  double fmin = 0;
  double fmax = 200;
  MPC mpc(alpha,
          state_weight,
          mu,
          fmin,
          fmax,
          std::make_unique<BrickState>(quad_state),
          std::make_unique<BrickModel>(quad_model),
          PARTIAL_CONDENSING_HPIPM,
          MPC_PREDICTION_HORIZON,
          "SPEED_ABS",
          0);

  // GaitSequencer
  // Gait gait( 1.0, 1.0, {0, 0, 0, 0}, MPC_DT,"test_gait_stand");
  // Gait gait( 0.5, 0.75, {0, 0.75, 0.5, 0.25}, MPC_DT,"test_gait_stepdance");
  // Gait gait(0.3, 0.5, {0, 0.5, 0.5, 0.0}, MPC_DT, "test_gait_trotting");
  SimpleGaitSequencer smqs(GaitDatabase::getGait(GaitDatabase::STATIC_WALK, MPC_DT),
                           0.03,
                           {
                               Eigen::Vector3d(0.167, 0.1538, 0.0),
                               Eigen::Vector3d(0.167, -0.1538, 0.0),
                               Eigen::Vector3d(-0.197, 0.1538, 0.0),
                               Eigen::Vector3d(-0.197, -0.1538, 0.0),
                           },
                           std::make_unique<BrickState>(quad_state),
                           std::make_unique<BrickModel>(quad_model),
                           1,
                           false,
                           false,
                           0.1,
                           0.26,
                           0.1,
                           false);

  //  BioGaitSequencer smqs(0.03,
  //                        {
  //                            Eigen::Vector3d(0.167, 0.1538, 0.0),
  //                            Eigen::Vector3d(0.167, -0.1538, 0.0),
  //                            Eigen::Vector3d(-0.197, 0.1538, 0.0),
  //                            Eigen::Vector3d(-0.197, -0.1538, 0.0),
  //                        },
  //                        std::make_unique<BrickState>(quad_state),
  //                        std::make_unique<BrickModel>(quad_model),
  //                        10,
  //                        false,
  //                        0.1,
  //                        0.26,
  //                        0.1,
  //                        false);

  // AdaptiveGait gait({0.0, 0.5, 0.5, 0.0}, MPC_DT, 0.15, 20, 0.03, 0.08);
  // AdaptiveGaitSequencer smqs(gait,
  //                            0.03,
  //                            {
  //                                Eigen::Vector3d(0.167, 0.1538, 0.0),
  //                                Eigen::Vector3d(0.167, -0.1538, 0.0),
  //                                Eigen::Vector3d(-0.197, 0.1538, 0.0),
  //                                Eigen::Vector3d(-0.197, -0.1538, 0.0),
  //                            },
  //                            std::make_unique<BrickState>(quad_state),
  //                            std::make_unique<BrickModel>(quad_model),
  //                            10,
  //                            true,
  //                            0.1,
  //                            0.26,
  //                            0.1,
  //                            false);

  Target target;
  target.active.hybrid_x_dot = true;
  target.active.hybrid_y_dot = true;
  target.active.z = true;
  target.active.x = false;
  target.active.y = false;
  target.active.z_dot = false;
  target.active.full_orientation = false;
  target.active.wx = true;
  target.active.wy = true;
  target.active.wz = true;
  target.hybrid_x_dot = 0.1;
  target.hybrid_y_dot = 0;
  target.z_dot = 0;

  target.full_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  target.x = quad_state.position_.x();
  target.y = quad_state.position_.y();
  target.z = quad_state.position_.z();

  smqs.UpdateTarget(target);

  GaitSequence gs;
  WrenchSequence ws;
  MPCPrediction prediction;
  FeetTargets ft;
  sim.SetFootPositions(quad_state.virt_feet_positions_);

  bool run = true;
  std::cout << '\n' << "Press a key to continue ([s] for stepping)" << std::endl;
  char mode = std::cin.get();

  // select plot type to be displayed
  enum PLOT_TYPE { NONE, GAIT_SEQUENCE, SOLVE_TIME };
  PLOT_TYPE plot_type = NONE;

  // prepare the different plots:
  // create figure for plotting gait seq
  auto f = matplot::figure(true);
  matplot::hold(false);
  auto ax_gait_plot = matplot::gca();
  ax_gait_plot->xlabel("Timestep");
  ax_gait_plot->ylabel("Leg index");
  std::vector<double> x = matplot::linspace(0, GAIT_SEQUENCE_SIZE - 1, GAIT_SEQUENCE_SIZE);

  // create solve time plot
  double buffer[BUFFER_SIZE];
  int buffer_index = 0;

  // adjust plot settings here
  int plot_after_iterations = 100;
  double y_limit_current = 20;
  double y_limit_average = 10;

  auto solve_time_fig = matplot::figure(true);
  solve_time_fig->size(1500, 750);
  matplot::hold(false);

  auto ax1 = matplot::subplot(2, 1, 0);
  ax1->title("Current Solve Time in ms");
  ax1->xlabel("Number of Iterations");
  ax1->ylabel("Time in ms");
  ax1->xlim({0, BUFFER_SIZE});
  ax1->ylim({0, y_limit_current});

  auto ax2 = matplot::subplot(2, 1, 1);
  ax2->title("Average Solve Time in ms");
  ax2->ylabel("Time in ms");
  ax2->ylim({0, y_limit_average});

  std::vector<double> plot_data(BUFFER_SIZE);
  std::vector<double> mean_plot;

  matplot::grid(ax1, true);
  matplot::grid(ax2, true);

  unsigned int it = 0;

  Eigen::Vector3d disturbance = Eigen::Vector3d::Zero();

  do {
    auto tic_gs = std::chrono::steady_clock::now();
    smqs.UpdateState(quad_state);
    smqs.GetGaitSequence(gs);
    auto toc_gs = std::chrono::steady_clock::now();
    auto tic_mpc = std::chrono::steady_clock::now();
    mpc.UpdateState(quad_state);
    mpc.UpdateGaitSequence(gs);
    static SolverInformation solver_info;
    mpc.GetWrenchSequence(ws, prediction, solver_info);
    auto toc_mpc = std::chrono::steady_clock::now();
    auto tic_sc_in = std::chrono::steady_clock::now();
    sc.UpdateState(quad_state);
    sc.UpdateGaitSequence(gs);

    switch (plot_type) {
      case NONE:
        break;

      case GAIT_SEQUENCE:
        // only update every 10 steps in continuous mode
        if (it % 10 == 0 || mode == 's') {
          // create array for plotting gait
          std::array<std::array<double, GAIT_SEQUENCE_SIZE>, N_LEGS> seq;
          for (int i = 0; i < GAIT_SEQUENCE_SIZE; ++i) {
            for (int leg = 0; leg < N_LEGS; ++leg) {
              seq[leg][i] = 255.0 * (double)(!gs.contact_sequence[i][leg]);
            }
          }
          ax_gait_plot->image(seq, seq, seq);  // use rgb image to ensure color even when all states the same
          f->draw();
        }
        break;

      case SOLVE_TIME:
        // write to buffer array in a cyclic manner
        buffer[buffer_index] = solver_info.total_solver_time * 1000;
        buffer_index++;
        buffer_index = buffer_index % BUFFER_SIZE;

        // only refresh the plot after some number of iterations
        if (it % plot_after_iterations == 0) {
          double current_elem = 0.0;
          int num_elements_for_mean_calc = 0;
          double sum = 0.0;
          for (int i = 0; i < BUFFER_SIZE; ++i) {
            current_elem = buffer[(i + buffer_index) % BUFFER_SIZE];
            plot_data.push_back(current_elem);
            // only consider elements, which are not zero for mean calculation
            if (current_elem > 0) {
              sum += current_elem;
              num_elements_for_mean_calc++;
            }
          }
          // ignore the first 100 values after startup
          if (it > 100) {
            double mean = static_cast<double>(sum / num_elements_for_mean_calc);
            mean_plot.push_back(mean);
            // plot average solve time
            ax2->plot(mean_plot)->line_width(2);
          }
          // plot current solve time
          ax1->plot(plot_data)->line_width(2);
          solve_time_fig->draw();
          // empty the vector after plotting
          plot_data.clear();
        }
        break;
    }

    it++;

    auto toc_sc_in = std::chrono::steady_clock::now();

    if (mode == 's') {
      for (unsigned int k = 0; k < MPC_PREDICTION_HORIZON; k++) {
        std::cout << "k=" << k << " :\t";
        for (unsigned int foot_id = 0; foot_id < N_LEGS; foot_id++) {
          std::cout << " (" << std::fixed << std::setprecision(3) << ws.forces[k][foot_id].x() << ", "
                    << ws.forces[k][foot_id].y() << ", " << ws.forces[k][foot_id].z() << ")";
          if ((abs(ws.forces[k][foot_id].x()) > (mu * ws.forces[k][foot_id].z()) + 0.1)
              || (abs(ws.forces[k][foot_id].y()) > (mu * ws.forces[k][foot_id].z()) + 0.1)) {
            std::cout << " - VIOLATION!";
          }
        }
        std::cout << "[" << gs.contact_sequence[k][0] << "," << gs.contact_sequence[k][1] << ","
                  << gs.contact_sequence[k][2] << "," << gs.contact_sequence[k][3] << "]   -  ";

        for (unsigned int foot_id = 0; foot_id < N_LEGS; foot_id++) {
          std::cout << " (" << std::fixed << std::setprecision(3) << gs.foot_position_sequence[k][foot_id].x() << ", "
                    << gs.foot_position_sequence[k][foot_id].y() << ", " << gs.foot_position_sequence[k][foot_id].z()
                    << ")";
        }

        std::cout << std::endl;
      }
      for (unsigned int k = 0; k <= MPC_PREDICTION_HORIZON; k++) {
        std::cout << " \t [";
        for (unsigned int state_id = 0; state_id < 13; state_id++) {
          std::cout << std::fixed << std::setprecision(4) << prediction.raw_data[k](state_id) << ", \t\t";
        }
        std::cout << "]" << std::endl;
      }
    }

    sim.SetGateSequence(gs);
    prediction.position[0] = quad_state.position_;
    prediction.orientation[0] = quad_state.orientation_;
    prediction.angular_velocity[0] = quad_state.angular_vel_;
    prediction.linear_velocity[0] = quad_state.linear_vel_;

    sim.SetWrenchSequence(ws, gs, prediction, disturbance);
    sim.SetMPCStatePrediction(prediction);

    std::cout << "GS Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(toc_gs - tic_gs).count() << "[ms]"
              << std::endl;
    std::cout << "MPC Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(toc_mpc - tic_mpc).count()
              << "[ms]" << std::endl;
    std::cout << "SC IN Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(toc_sc_in - tic_sc_in).count()
              << "[ms]" << std::endl;
    if (mode == 's') {
      do {
        std::cout << '\n' << "Press a key to continue...";
      } while (std::cin.get() != '\n');
    }

    unsigned int sim_steps = CONTROL_DT / SWING_LEG_DT;
    for (unsigned int sim_step = 0; sim_step < sim_steps; sim_step++) {
      auto tic_sc_out = std::chrono::steady_clock::now();
      sc.UpdateState(quad_state);
      sc.GetFeetTargets(ft);
      auto toc_sc_out = std::chrono::steady_clock::now();
      for (unsigned int foot_idx = 0; foot_idx < N_LEGS; foot_idx++) {
        if (gs.contact_sequence[0][foot_idx]) {  // Replace the current active feet with actual position
          ft.positions[foot_idx] = quad_model.GetFootPositionInWorld(foot_idx, quad_state);
        }
      }
      std::cout << "SC OUT Time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(toc_sc_out - tic_sc_out).count() << "[ms]"
                << std::endl;
      sim.SetFootPositions(ft.positions);
      // Get command
      auto gp = sim.GetGamepad();
      if (gp.axes.size() == 4) {
        target.hybrid_x_dot = -gp.axes[1];
        target.hybrid_y_dot = -gp.axes[0];
        target.wz = -gp.axes[2];
        disturbance.x() = (gp.button_values[3] - gp.button_values[0]) * 40;
        disturbance.y() = (gp.button_values[2] - gp.button_values[1]) * 40;
        smqs.UpdateTarget(target);
        std::cout << "Disturbance: [" << disturbance.x() << ", " << disturbance.y() << ", 0.0]" << std::endl;
        std::cout << "Update Target: " << target.hybrid_x_dot << ", " << target.hybrid_y_dot << "," << target.wz
                  << std::endl;
      } else {
        std::cout << "Old Target: " << target.hybrid_x_dot << ", " << target.hybrid_y_dot << "," << target.wz
                  << std::endl;
      }
      sim.Simulate(SWING_LEG_DT);
      sim.GetState(quad_state, gs.contact_sequence[0]);
    }

  } while (run);
}
