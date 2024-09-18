cmake_minimum_required(VERSION 3.8)
project(state_estimation)

set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_STANDARD 17)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

option(WITH_VICON "Use Vicon" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(inekf)
if (WITH_VICON)
    find_package(vicon_receiver REQUIRED)
endif ()
# Add the executable for the C++ node
add_executable(state_estimation src/state_estimation_node.cpp src/contact_detection.cpp src/kalman_filter.cpp)

ament_target_dependencies(state_estimation rclcpp sensor_msgs interfaces inekf std_srvs)
if (WITH_VICON)
    ament_target_dependencies(state_estimation vicon_receiver)
    target_compile_definitions(state_estimation PUBLIC WITH_VICON)
endif ()

target_link_libraries(state_estimation common::quad_model_symbolic)

target_include_directories(
        state_estimation
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(TARGETS state_estimation DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
