cmake_minimum_required(VERSION 3.8)

# ------------------------------------------------------------
# Project declaration
# ------------------------------------------------------------
# Defines the project name and enables CMake features
project(aqua_slam_ros2)

# ------------------------------------------------------------
# Compiler settings
# ------------------------------------------------------------

# Suppress warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile_commands.json (useful for clangd / VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define compile-time macro
add_definitions(-DCOMPILEDWITHC11)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ------------------------------------------------------------
# Find required packages (ROS 2 uses ament instead of catkin)
# ------------------------------------------------------------

find_package(ament_cmake REQUIRED)

# ROS 2 dependencies (converted from ROS 1 equivalents)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)           # DVL msgs from Barcelona
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)

# Third-party dependencies
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(PCL REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS log_setup log serialization filesystem system iostreams)

# Enable Boost dynamic linking
add_definitions(-DBOOST_LOG_DYN_LINK)

# ------------------------------------------------------------
# Include directories
# ------------------------------------------------------------
# Makes header files available to targets
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${PROJECT_SOURCE_DIR}/include/cv_bridge_slam
  ${PROJECT_SOURCE_DIR}/Thirdparty/octomap/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ------------------------------------------------------------
# Add third-party subdirectories (built as part of project)
# ------------------------------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/octomap)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/Sophus)

# ------------------------------------------------------------
# Build main SLAM library
# ------------------------------------------------------------
# This builds your core ORB-DVL shared library

add_library(${PROJECT_NAME} SHARED
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    src/ORBextractor.cc
    src/Converter.cc
    src/ImuTypes.cc
    src/DVLGroPreIntegration.cpp
    src/Frame.cc
    src/KeyFrame.cc
    src/KeyFrameDatabase.cc
    src/Map.cc
    src/MapPoint.cc
    src/TwoViewReconstruction.cc
    src/Atlas.cc
    src/MapDrawer.cc
    src/Sim3Solver.cc
    src/PnPsolver.cc
    src/MLPnPsolver.cpp
)

# Link libraries to core SLAM library
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  fmt::fmt
  DBoW2
  g2o
  octomap
  ${Pangolin_LIBRARIES}
)

# Add ROS 2 dependencies (replaces catkin_LIBRARIES)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  dvl_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  image_transport
  message_filters
  cv_bridge
)

# ------------------------------------------------------------
# Optimization library
# ------------------------------------------------------------

add_library(${PROJECT_NAME}_Optimization SHARED
    src/OptimizableTypes.cpp
    src/ImuTypes.cc
    src/DVLGroPreIntegration.cpp
    src/G2oTypes.cc
)

target_link_libraries(${PROJECT_NAME}_Optimization
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  fmt::fmt
  DBoW2
  g2o
  sophus
)

ament_target_dependencies(${PROJECT_NAME}_Optimization
  rclcpp
)

# ------------------------------------------------------------
# Executable node
# ------------------------------------------------------------

# Create ROS 2 node executable
add_executable(stereo_dvl2 src/ros_stereo_DVL_tightly.cc)

# Link executable to internal libraries
target_link_libraries(stereo_dvl2
  ${PROJECT_NAME}
  ${PROJECT_NAME}_Optimization
  ${Boost_LIBRARIES}
)

# Add ROS 2 dependencies
ament_target_dependencies(stereo_dvl2
  rclcpp
  sensor_msgs
  std_msgs
  dvl_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ------------------------------------------------------------
# Installation rules (ROS 2 style)
# ------------------------------------------------------------

# Install executable into ROS 2 install space
install(TARGETS
  stereo_dvl2
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_Optimization
  DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# ------------------------------------------------------------
# Export package to ROS 2 build system
# ------------------------------------------------------------

ament_package()