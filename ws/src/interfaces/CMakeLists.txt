cmake_minimum_required(VERSION 3.8)
project(interfaces)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files "msg/JointCmd.msg"
        "msg/QuadState.msg"
        "msg/JointState.msg"
        "msg/LegCmd.msg"
        "msg/TrajectoryStatus.msg"
        "msg/QuadControlTarget.msg"
        "msg/PositionSequence.msg"
        "msg/VectorSequence.msg"
        "msg/GaitState.msg"
        "msg/ContactState.msg"
        "msg/WBCTarget.msg"
        "msg/BatteryState.msg"
        "msg/SimulationDisturbance.msg"
        "msg/GaitSequence.msg"
        "msg/QuadModel.msg"
        "msg/ControllerInfo.msg"
        "msg/WBCReturn.msg"
        "msg/MPCDiagnostics.msg"
)

set(srv_files "srv/EnableMotors.srv"
        "srv/DisableMotors.srv"
        "srv/ZeroMotors.srv"
        "srv/ResetSimulation.srv"
        "srv/StepSimulation.srv"
        "srv/KeepJointPositions.srv"
        "srv/ChangeLegDriverMode.srv"
        "srv/DisturbSim.srv")

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

rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        DEPENDENCIES
        std_msgs
        builtin_interfaces
        geometry_msgs
        sensor_msgs)

ament_package()
