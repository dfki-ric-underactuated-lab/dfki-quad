cmake_minimum_required(VERSION 3.8)
project(controllers)

option(ON_PI "On Raspberry Pi" FALSE)
if (ON_PI)
    message(WARNING "Skipping package on Raspberry Pi.")
    return()
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
else ()
    set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
    message(WARNING "detected non ARM architecture, applying -Ofast - this might be harmfull!")
endif ()


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP) # for speeding up eigens matrix multiplications
if (NOT WITHOUT_DRAKE)
    find_package(drake CONFIG REQUIRED)
    find_package(simulator REQUIRED)
endif ()
find_package(acados REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(unitree_go REQUIRED)
find_package(fmt REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OsqpEigen REQUIRED)


# ARCOPT
pkg_search_module(wbc-core REQUIRED IMPORTED_TARGET wbc-core)
pkg_search_module(wbc-robot_models-pinocchio REQUIRED IMPORTED_TARGET wbc-robot_models-pinocchio)
pkg_search_module(wbc-solvers-eiquadprog REQUIRED IMPORTED_TARGET wbc-solvers-eiquadprog)
pkg_search_module(wbc-solvers-qpswift REQUIRED IMPORTED_TARGET wbc-solvers-qpswift)
pkg_search_module(wbc-solvers-proxqp REQUIRED IMPORTED_TARGET wbc-solvers-proxqp)
pkg_search_module(wbc-solvers-qpoases REQUIRED IMPORTED_TARGET wbc-solvers-qpoases)
pkg_search_module(wbc-scenes-acceleration_reduced_tsid REQUIRED IMPORTED_TARGET wbc-scenes-acceleration_reduced_tsid)
pkg_search_module(wbc-scenes-acceleration_tsid REQUIRED IMPORTED_TARGET wbc-scenes-acceleration_tsid)
pkg_search_module(wbc-controllers REQUIRED IMPORTED_TARGET wbc-controllers)
pkg_search_module(wbc-solvers-osqp REQUIRED IMPORTED_TARGET wbc-solvers-osqp)
pkg_search_module(wbc-types REQUIRED IMPORTED_TARGET wbc-types)
pkg_search_module(wbc-solvers-hpipm REQUIRED IMPORTED_TARGET wbc-solvers-hpipm)
# pkg_search_module(base-types REQUIRED IMPORTED_TARGET base-types)


include_directories(include include/mit_controller src)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights comment the
    # line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo) comment the line
    # when this package is in a git repo and when a copyright and license is added
    # to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()


# ##############################################################################
# --- Trajectory Controller --- #
# ##############################################################################

if (NOT WITHOUT_DRAKE)
    add_executable(trajectory_replay src/trajectory_replay.cpp)

    ament_target_dependencies(trajectory_replay interfaces rclcpp
            sensor_msgs geometry_msgs common)

    target_include_directories(
            trajectory_replay
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
    install(TARGETS trajectory_replay DESTINATION lib/${PROJECT_NAME})

endif ()
# ##############################################################################
# --- Gait Plotting --- #
# ##############################################################################

add_executable(gait_plotting src/tools/plotting.cpp)

ament_target_dependencies(gait_plotting Eigen3 interfaces rclcpp common
)
target_link_libraries(gait_plotting matplot)

target_include_directories(
        gait_plotting
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ##############################################################################
# --- Gait Sequence Plotting --- #
# ##############################################################################

add_executable(gait_sequence_plotting src/tools/plot_gs.cpp)

ament_target_dependencies(gait_sequence_plotting Eigen3 interfaces rclcpp common
)
target_link_libraries(gait_sequence_plotting matplot)

target_include_directories(
        gait_sequence_plotting
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ##############################################################################
# --- MIT CONTROLLER--- #
# ##############################################################################
if (NOT WITHOUT_DRAKE)
    add_executable(mitcontroller
            src/mit_controller/mit_controller.cpp
            src/mit_controller/mpc.cpp
            src/mit_controller/gait.cpp
            src/mit_controller/simple_gait_sequencer.cpp
            src/mit_controller/bio_gait_sequencer.cpp
            src/mit_controller/adaptive_gait_sequencer.cpp
            src/mit_controller/mpc_trajectory_planner.cpp
            src/mit_controller/raibert_foot_step_planner.cpp
            src/mit_controller/swing_leg_controller.cpp
            src/potato_sim/potato_sim.cpp
            src/potato_sim/potato_model.cpp
            src/SwingTrajectory.cpp
            src/model_adaptation/least_squares_model_adaptation.cpp
    )

    ament_target_dependencies(mitcontroller Eigen3 interfaces common drake simulator)
    target_link_libraries(mitcontroller acados matplot)
    target_compile_options(mitcontroller PRIVATE -Wall -Wextra -Wpedantic -Werror)
    ament_export_targets(mitcontroller HAS_LIBRARY_TARGET)
    install(
            TARGETS mitcontroller
            EXPORT mitcontroller
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES
            DESTINATION include)
endif ()


# ##############################################################################
# --- MIT CONTROLLER NODE --- #
# ##############################################################################
add_executable(mitcontrollernode
        src/mit_controller_node.cpp
        src/mit_controller/mpc.cpp
        src/mit_controller/gait.cpp
        src/mit_controller/simple_gait_sequencer.cpp
        src/mit_controller/adaptive_gait_sequencer.cpp
        src/mit_controller/bio_gait_sequencer.cpp
        src/mit_controller/mpc_trajectory_planner.cpp
        src/mit_controller/raibert_foot_step_planner.cpp
        src/mit_controller/swing_leg_controller.cpp
        src/mit_controller/inverse_dynamics.cpp
        src/mit_controller/wbc_arc_opt.cpp
        src/model_adaptation/least_squares_model_adaptation.cpp
        src/SwingTrajectory.cpp
)

set_source_files_properties(
        src/mit_controller/wbc_arc_opt.cpp
        PROPERTIES
        COMPILE_FLAGS -fpermissive
)

target_link_libraries(mitcontrollernode
        PkgConfig::wbc-core
        PkgConfig::wbc-robot_models-pinocchio
        PkgConfig::wbc-solvers-eiquadprog
        PkgConfig::wbc-solvers-qpswift
        PkgConfig::wbc-solvers-proxqp
        PkgConfig::wbc-solvers-qpoases
        PkgConfig::wbc-solvers-osqp
        PkgConfig::wbc-scenes-acceleration_reduced_tsid
        PkgConfig::wbc-scenes-acceleration_tsid
        PkgConfig::wbc-controllers
        PkgConfig::wbc-types
        OsqpEigen::OsqpEigen
        PkgConfig::wbc-solvers-hpipm
)

if (ROBOT_NAME STREQUAL "ulab")
    target_compile_definitions(mitcontrollernode PUBLIC ROBOT_MODEL=ULAB)
    MESSAGE(WARNING "Building for ulab quad")
elseif (ROBOT_NAME STREQUAL "go2")
    target_compile_definitions(mitcontrollernode PUBLIC ROBOT_MODEL=GO2)
    MESSAGE(WARNING "Building for go2 quad")
else ()
    MESSAGE(FATAL_ERROR "Please specify robot model during build")
endif ()

target_compile_options(mitcontrollernode PRIVATE -Wall -Wextra -Wpedantic -Werror)
ament_target_dependencies(mitcontrollernode Eigen3 common interfaces rclcpp)
target_link_libraries(mitcontrollernode acados fmt::fmt)
target_include_directories(
        mitcontrollernode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# --- test Swing leg Controller --- #
# ##############################################################################

add_executable(testSwingTrajectoryControllerNode src/testSwingTrajectoryControllerNode.cpp
        src/SwingTrajectory.cpp)

ament_target_dependencies(testSwingTrajectoryControllerNode Eigen3 interfaces rclcpp
        sensor_msgs geometry_msgs common)

target_include_directories(
        testSwingTrajectoryControllerNode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# ##############################################################################
# --- Sys ID Leg Controller --- #
# ##############################################################################

add_executable(SysIDLegControllerNode src/system_identification/SysIDLegControllerNode.cpp src/system_identification/SysIDTrajectory.cpp)

ament_target_dependencies(SysIDLegControllerNode Eigen3 interfaces rclcpp common)

target_include_directories(
        SysIDLegControllerNode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# --- CPU power consumption logging--- #
# ##############################################################################
add_executable(log_cpu_power src/tools/log_cpu_power.cpp)

ament_target_dependencies(log_cpu_power
        rclcpp
        std_msgs
)

target_include_directories(
        log_cpu_power
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# Install nodes #
# ##############################################################################

##############################
# Python package and scripts #
##############################

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/joy_to_target.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/quad_stand_up.py DESTINATION lib/${PROJECT_NAME})

#########
# Share #
#########

if (NOT WITHOUT_DRAKE)
    install(TARGETS trajectory_replay gait_plotting gait_sequence_plotting log_cpu_power testSwingTrajectoryControllerNode SysIDLegControllerNode mitcontrollernode DESTINATION lib/${PROJECT_NAME})
else ()
    install(TARGETS gait_plotting gait_sequence_plotting log_cpu_power testSwingTrajectoryControllerNode SysIDLegControllerNode mitcontrollernode DESTINATION lib/${PROJECT_NAME})
endif ()

install(DIRECTORY launch config trajectories DESTINATION share/${PROJECT_NAME}/)

ament_package()
