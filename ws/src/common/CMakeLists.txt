cmake_minimum_required(VERSION 3.8)
project(common)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

if (ON_PI)
    set(WITHOUT_DRAKE ON)
endif ()

if (NOT WITHOUT_DRAKE)
    find_package(drake CONFIG REQUIRED /opt/drake)
endif ()
include_directories(include/common)
if (NOT WITHOUT_DRAKE)
    add_library(quad_model src/quad_model.cpp src/quad_state.cpp)
endif ()
add_library(quad_model_symbolic src/quad_state.cpp src/quad_model_symbolic.cpp)
if (ROBOT_NAME STREQUAL "ulab")
    target_compile_definitions(quad_model_symbolic PUBLIC ROBOT_MODEL=ULAB)
    MESSAGE(NOTICE "Building for ulab quad")
elseif (ROBOT_NAME STREQUAL "go2")
    target_compile_definitions(quad_model_symbolic PUBLIC ROBOT_MODEL=GO2)
    MESSAGE(NOTICE "Building for go2 quad")
else ()
    MESSAGE(WARNING "No default constructor!")
endif ()
if (NOT WITHOUT_DRAKE)
    ament_target_dependencies(quad_model drake Eigen3 interfaces rclcpp)
endif ()
ament_target_dependencies(quad_model_symbolic interfaces rclcpp)
target_link_libraries(quad_model_symbolic Eigen3::Eigen)


target_include_directories(
        quad_model_symbolic
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if (NOT WITHOUT_DRAKE)
    target_include_directories(
            quad_model_symbolic
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
endif ()
if (NOT WITHOUT_DRAKE)
    ament_export_targets(quad_model HAS_LIBRARY_TARGET)
endif ()
ament_export_targets(quad_model_symbolic HAS_LIBRARY_TARGET)

install(DIRECTORY include/common DESTINATION include)

if (NOT WITHOUT_DRAKE)
    install(
            TARGETS quad_model
            EXPORT quad_model
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES
            DESTINATION include)
endif ()

install(
        TARGETS quad_model_symbolic
        EXPORT quad_model_symbolic
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include)

if (NOT WITHOUT_DRAKE)
    add_executable(model_test src/tests/model_test.cpp)

    ament_target_dependencies(model_test interfaces drake rclcpp)
    target_link_libraries(model_test Eigen3::Eigen quad_model_symbolic)
    target_include_directories(
            model_test
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
endif ()
##############################
# Python package and scripts #
##############################

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/plot_trajectory_bagfile.py DESTINATION lib/${PROJECT_NAME})

#########
# Share #
#########

install(DIRECTORY include model launch DESTINATION share/${PROJECT_NAME})

ament_package()

