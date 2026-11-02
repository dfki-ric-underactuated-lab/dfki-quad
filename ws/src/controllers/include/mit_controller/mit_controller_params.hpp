#pragma once

// MPC prediction horizon (number of stages). Set at build time with
//   colcon build --cmake-args -DDFKI_QUAD_MPC_HORIZON=<N>
// (see CMakeLists.txt); defaults to 10.
#ifndef DFKI_QUAD_MPC_HORIZON
#define DFKI_QUAD_MPC_HORIZON 10
#endif

static constexpr int N_LEGS = 4;
static constexpr int N_JOINTS_PER_LEG = 3;
static constexpr int GAIT_SEQUENCE_SIZE = 100;
static constexpr int MPC_PREDICTION_HORIZON = DFKI_QUAD_MPC_HORIZON;
static_assert(MPC_PREDICTION_HORIZON >= 1, "DFKI_QUAD_MPC_HORIZON must be at least 1");
static_assert(MPC_PREDICTION_HORIZON + 1 <= GAIT_SEQUENCE_SIZE, "the gait sequence must cover the MPC horizon");
static constexpr double MPC_DT = 0.05;
static constexpr double MPC_CONTROL_DT = 0.01;
static constexpr double SWING_LEG_DT = 0.002;
static constexpr double CONTROL_DT = 0.002;
static constexpr double WBC_CYCLE_DT = CONTROL_DT;
static constexpr double MODEL_ADAPTATION_DT = 0.01;
static constexpr double MODEL_ADAPTATION_BATCH_SIZE = 100;

static constexpr int FEET_POSITION_SEQUENCE_SIZE = int((MPC_DT / MPC_CONTROL_DT) * GAIT_SEQUENCE_SIZE);

// message publishers
static const bool PUBLISH_SWING_LEG_TRAJECTORIES = false;
static const bool PUBLISH_GAIT_STATE = true;
static const bool PUBLISH_OPEN_LOOP_TRAJECTORY = true;
static const bool PUBLISH_SOLVE_TIME = true;
static const bool PUBLISH_WBC_SOLVE_TIME = true;
static const bool PUBLISH_WBC_TARGET = true;
static const bool PUBLISH_GAIT_SEQUENCE = true;
static const bool PUBLISH_HEARTBEAT = true;

#ifdef ROBOT_MODEL
#if ROBOT_MODEL == GO2
static constexpr bool USE_WBC = true;
#elif ROBOT_MODEL == ULAB
static constexpr bool USE_WBC = false;
#endif
#endif

// #define DEBUG_PRINTS
