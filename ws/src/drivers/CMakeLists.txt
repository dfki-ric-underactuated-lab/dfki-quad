cmake_minimum_required(VERSION 3.8)
project(drivers)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
option(ON_PI "On Raspberry Pi" FALSE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(include/drivers)

if (ROBOT_NAME STREQUAL "ulab")
    message(STATUS "Selected robot: ${ROBOT_NAME}")
    if (NOT ON_PI OR NOT WITHOUT_DRAKE)
        # not building on raspberry pi
        find_package(drake CONFIG REQUIRED /opt/drake)
    endif ()

    # Add include directories
    # include_directories(include/drivers)
    # Add shared library mjbots_api from /usr/local/lib NOTE: Make sure you have the
    # shared library path in LD_LIBRARY_PATH
    include_directories(/usr/local/lib)
    include_directories(/home/ubuntu/eigen/Eigen/Dense)

    if (ON_PI)
        # building on raspberry pi
        find_library(MJBOTS_API NAMES mjbots_api REQUIRED)
        #message(${MJBOTS_API})

        # Example commands for motor drivers inheriting from AbstractMotorDriver
        # add_executable(example_driver src/motor_driver_example.cpp
        # src/abstract_motor_driver.cpp) ament_target_dependencies(example_driver rclcpp
        # geometry_msgs sensor_msgs std_msgs dfki_quad_interfaces)

        #add_executable(test_abstract_motor_driver src/test_abstract_motor_driver.cpp
        #                                          src/abstract_motor_driver.cpp)
        #ament_target_dependencies(test_abstract_motor_driver rclcpp geometry_msgs
        #                          sensor_msgs std_msgs interfaces common)

        add_executable(mjbots_ros2_motor_driver src/mjbots_ros2_motor_driver.cpp
                src/abstract_motor_driver.cpp)
        ament_target_dependencies(mjbots_ros2_motor_driver rclcpp geometry_msgs
                sensor_msgs std_msgs interfaces common)
        target_link_libraries(mjbots_ros2_motor_driver ${MJBOTS_API})

        install(TARGETS mjbots_ros2_motor_driver DESTINATION lib/${PROJECT_NAME})
    endif ()

elseif (ROBOT_NAME STREQUAL "go2")
    message("Selected robot: ${ROBOT_NAME}")
    find_package(unitree_sdk2 REQUIRED)
    find_package(unitree_go REQUIRED)
    find_package(unitree_api REQUIRED)
    find_package(unitree_ros2_example REQUIRED)
    find_package(CycloneDDS REQUIRED)
    find_package(CycloneDDS-CXX REQUIRED)

    #add_library(unitree_sdk2 STATIC IMPORTED) # or STATIC instead of SHARED
    #set_target_properties(unitree_sdk2 PROPERTIES
    #        IMPORTED_LOCATION "/usr/local/lib/libunitree_sdk2.a"
    #        INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include/"
    #)
    #    add_library(ddscxx SHARED IMPORTED) # or STATIC instead of SHARED
    #    set_target_properties(ddscxx PROPERTIES
    #            IMPORTED_LOCATION "/usr/local/lib/libddscxx.so"
    #            INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include/"
    #    )
    #add_library(ddsc SHARED IMPORTED) # or STATIC instead of SHARED
    #set_target_properties(ddsc PROPERTIES
    #        IMPORTED_LOCATION "/usr/local/lib/libddsc.so"
    #        INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include/"
    #)

    ########## UNITREE DRIVERS
    add_executable(unitree_ros2_motor_driver src/unitree_ros2_motor_driver.cpp
            src/abstract_motor_driver.cpp
    )
    ament_target_dependencies(unitree_ros2_motor_driver rclcpp geometry_msgs
            sensor_msgs std_msgs interfaces unitree_ros2_example unitree_sdk2 unitree_go unitree_api)
    target_link_libraries(unitree_ros2_motor_driver common::quad_model_symbolic CycloneDDS-CXX::ddscxx CycloneDDS::ddsc rt pthread)


    install(TARGETS
            unitree_ros2_motor_driver
            DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

    install(TARGETS
            # unitree_read_motion_state
            # unitree_sport_mode_ctrl
            # unitree_read_low_state
            # unitree_low_level_ctrl
            unitree_ros2_motor_driver
            DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
else ()
    message("Unknown robot name: ${ROBOT_NAME}, just building leg driver")
endif ()

add_executable(leg_driver src/leg_driver.cpp)
ament_target_dependencies(leg_driver rclcpp interfaces Eigen3 std_srvs)
target_link_libraries(leg_driver common::quad_model_symbolic)
install(TARGETS leg_driver DESTINATION lib/${PROJECT_NAME})

add_executable(contact_sensor_driver src/contact_sensor_driver.cpp)
ament_target_dependencies(contact_sensor_driver rclcpp interfaces)
target_link_libraries(contact_sensor_driver ${LIBSERIAL_LIBRARIES} common::quad_model_symbolic)
install(TARGETS contact_sensor_driver DESTINATION lib/${PROJECT_NAME})

# Install executables
#install(TARGETS test_abstract_motor_driver DESTINATION lib/${PROJECT_NAME})
# install(TARGETS mjbots_ros2_motor_driver DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories install(DIRECTORY launch DESTINATION
# share/${PROJECT_NAME}/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
