cmake_minimum_required(VERSION 3.8)
project(iris_vslam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ORB-SLAM3 paths
set(ORB_SLAM3_ROOT_DIR "$ENV{HOME}/ORB_SLAM3")

#--------------------------------------------------------------------------------------------
# Find dependencies
#--------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

#--------------------------------------------------------------------------------------------
# Include directories
#--------------------------------------------------------------------------------------------
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

#--------------------------------------------------------------------------------------------
# C++ executable: slam_node
#--------------------------------------------------------------------------------------------
add_executable(slam_node src/slam_node.cpp)

ament_target_dependencies(slam_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
  nav_msgs
)

target_link_libraries(slam_node
  ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  -lboost_serialization
  -lcrypto
)

#--------------------------------------------------------------------------------------------
# Install C++ executable
#--------------------------------------------------------------------------------------------
install(TARGETS slam_node
  DESTINATION lib/${PROJECT_NAME}
)

#--------------------------------------------------------------------------------------------
# Install Python scripts
#--------------------------------------------------------------------------------------------
install(PROGRAMS
  ${PROJECT_NAME}/slam_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

#--------------------------------------------------------------------------------------------
# Install launch and config files
#--------------------------------------------------------------------------------------------
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()