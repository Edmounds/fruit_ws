cmake_minimum_required(VERSION 3.16)
project(glim_ros VERSION 1.2.2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" OFF)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(glim REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

if(BUILD_WITH_CUDA)
  add_definitions(-DBUILD_GTSAM_POINTS_GPU)
endif()

### glim_ros ###
ament_auto_add_library(glim_ros SHARED
  src/glim_ros/glim_ros.cpp
  src/glim_ros/ros_qos.cpp
)
target_include_directories(glim_ros PUBLIC
  include
)
target_link_libraries(glim_ros
  glim::glim
)
rclcpp_components_register_nodes(glim_ros "glim::GlimROS")

ament_auto_add_library(rviz_viewer SHARED
  src/glim_ros/rviz_viewer.cpp
)

### glim_rosnode ###
ament_auto_add_executable(glim_rosnode
  src/glim_rosnode.cpp
)
target_link_libraries(glim_rosnode
  glim_ros
)

### glim_rosbag ###
ament_auto_add_executable(glim_rosbag
  src/glim_rosbag.cpp
)
target_link_libraries(glim_rosbag
  glim_ros
)

### validator_node ###
ament_auto_add_executable(validator_node
  src/validator_node.cpp
)
target_link_libraries(validator_node
  glim_ros
)

if(BUILD_WITH_VIEWER)
  ### offline_viewer ###
  ament_auto_add_executable(offline_viewer
    src/offline_viewer.cpp
  )
  target_include_directories(offline_viewer PUBLIC
    include
  )
  target_link_libraries(offline_viewer
    glim::interactive_viewer
    Boost::program_options
  )

  ### map_editor ###
  ament_auto_add_executable(map_editor
    src/map_editor.cpp
  )
  target_link_libraries(map_editor
    glim::map_editor
    Boost::program_options
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(Eigen3 REQUIRED)

  ament_add_gtest(base_frame_alignment_test test/base_frame_alignment_test.cpp)
  target_include_directories(base_frame_alignment_test PRIVATE include)
  target_link_libraries(base_frame_alignment_test Eigen3::Eigen)

  ament_add_pytest_test(runtime_contract test/test_runtime_contract.py)
endif()

ament_auto_package(INSTALL_TO_SHARE launch rviz)
