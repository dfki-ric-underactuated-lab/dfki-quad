cmake_minimum_required(VERSION 3.8)
project(simulator)

option(ON_PI "On Raspberry Pi" FALSE)
if (ON_PI)
    message(WARNING "Skipping package on Raspberry Pi.")
    return()
endif ()

if (WITHOUT_DRAKE)
    message(WARNING "Skipping packages because it depends on drake")
    return()
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_BUILD_TYPE Release)
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drake CONFIG REQUIRED)
find_package(rosgraph_msgs CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(${EIGEN_INCLUDE_DIR})

add_executable(visualizer src/drake_visualizer.cpp)
add_executable(simulator src/drake_simulator.cpp src/pd_changeable_controller.cpp)
add_executable(sim_disturber src/disturbance_node.cpp)
add_library(drake_visualizers src/drake_vizualisers/positions_vizualiser.cpp
        src/drake_vizualisers/forces_vizualiser.cpp
        src/drake_vizualisers/vectors_vizualiser.cpp
        src/drake_vizualisers/poses_vizualiser.cpp
)

target_link_libraries(visualizer drake_visualizers)

include_directories(include)


ament_target_dependencies(
        visualizer
        rclcpp
        interfaces
        common
        sensor_msgs
        drake
        Eigen3)

ament_target_dependencies(drake_visualizers
        drake
        Eigen3
)

ament_target_dependencies(
        simulator
        drake
        rclcpp
        interfaces
        sensor_msgs
        rosgraph_msgs
        common
        Eigen3
)

ament_target_dependencies(
        sim_disturber
        rclcpp
        interfaces
        common
        Eigen3
)

target_include_directories(
        visualizer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(simulator
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/**>
        $<INSTALL_INTERFACE:include>)

target_include_directories(
        drake_visualizers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(
        sim_disturber PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_export_targets(drake_visualizers HAS_LIBRARY_TARGET)


install(TARGETS visualizer DESTINATION lib/${PROJECT_NAME})
install(TARGETS simulator DESTINATION lib/${PROJECT_NAME})
install(TARGETS sim_disturber DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/drake_vizualizers DESTINATION include)

install(
        TARGETS drake_visualizers
        EXPORT drake_visualizers
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include)

install(DIRECTORY include launch DESTINATION share/${PROJECT_NAME})

ament_package()
