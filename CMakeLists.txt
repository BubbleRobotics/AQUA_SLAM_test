cmake_minimum_required(VERSION 3.8)

# ------------------------------------------------------------
# Project declaration
# ------------------------------------------------------------
project(aqua_slam_ros2)

# ------------------------------------------------------------
# Compiler settings
# ------------------------------------------------------------
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DCOMPILEDWITHC11)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ------------------------------------------------------------
# Find required packages (ROS 2 and third-party)
# ------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

# Third-party dependencies
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(PCL REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS log_setup log serialization filesystem system iostreams)

add_definitions(-DBOOST_LOG_DYN_LINK)

# ------------------------------------------------------------
# Third-party subdirectories built as part of project
# ------------------------------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/octomap)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/Sophus)

# ------------------------------------------------------------
# Include directories
# ------------------------------------------------------------
# Properly propagate local and third-party headers
set(AQUA_SLAM_INCLUDES
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/CameraModels>
  $<INSTALL_INTERFACE:include/CameraModels>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/cv_bridge_slam>
  $<INSTALL_INTERFACE:include/cv_bridge_slam>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/octomap/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/g2o>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/Sophus>
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ------------------------------------------------------------
# Build main SLAM library
# ------------------------------------------------------------
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
    src/LKTracker.cpp
    src/LKTrackerNode.cpp
    src/LoopClosing.cc
    src/System.cc
    src/RosHandling.cpp
    src/DenseMapper.cpp
    src/LocalMapping.cc
    src/DvlGyroOptimizer.cpp
    src/FrameDrawer.cc
    src/Initializer.cc
    src/Integrator.cpp
    src/Tracking.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC ${AQUA_SLAM_INCLUDES})

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  fmt::fmt
  DBoW2
  g2o
  octomap
  ${Pangolin_LIBRARIES}
  # sensor_msgs::sensor_msgs
)

# Include the ros libraries that are needed
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
  std_srvs
  pcl_conversions
  octomap_msgs
  visualization_msgs
  pcl_ros
)

# ------------------------------------------------------------
# Optimization library
# ------------------------------------------------------------
add_library(${PROJECT_NAME}_Optimization SHARED
    src/OptimizableTypes.cpp
    src/ImuTypes.cc
    src/DVLGroPreIntegration.cpp
    src/G2oTypes.cc
    src/Optimizer.cc
)

# Include main library headers as well
target_include_directories(${PROJECT_NAME}_Optimization PUBLIC ${AQUA_SLAM_INCLUDES})
target_link_libraries(${PROJECT_NAME}_Optimization
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  fmt::fmt
  DBoW2
  g2o
  sophus
  # sensor_msgs::sensor_msgs
  # sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
)

# Include the ros libraries that are needed
ament_target_dependencies(${PROJECT_NAME}_Optimization
  rclcpp
  sensor_msgs
  std_msgs
  dvl_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  image_transport
  cv_bridge
  octomap_msgs
  std_srvs
  visualization_msgs
  pcl_ros
)

# ------------------------------------------------------------
# Executable node
# ------------------------------------------------------------
add_executable(stereo_dvl2 src/ros_stereo_DVL_tightly.cc)
target_include_directories(stereo_dvl2 PUBLIC ${AQUA_SLAM_INCLUDES})
target_link_libraries(stereo_dvl2
  ${PROJECT_NAME}
  ${PROJECT_NAME}_Optimization
  ${Boost_LIBRARIES}
)

# Include the ros libraries that are needed
ament_target_dependencies(
  stereo_dvl2
  rclcpp
  sensor_msgs
  std_msgs
  dvl_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  # sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
)

# ------------------------------------------------------------
# Installation rules
# ------------------------------------------------------------
install(TARGETS
  stereo_dvl2
  ${PROJECT_NAME}
  ${PROJECT_NAME}_Optimization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

# ------------------------------------------------------------
# Export package
# ------------------------------------------------------------
ament_package()