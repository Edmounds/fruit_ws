cmake_minimum_required(VERSION 3.16)
project(pb_gicp_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

include(FetchContent)
FetchContent_Declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp.git
  GIT_TAG v1.0.1
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(small_gicp)
if(NOT small_gicp_POPULATED)
  FetchContent_Populate(small_gicp)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

ament_auto_add_library(${PROJECT_NAME} SHARED src/gicp_localization.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${small_gicp_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX Eigen3::Eigen ${PCL_LIBRARIES})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN pb_gicp_localization::GicpLocalizationNode
  EXECUTABLE pb_gicp_localization_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registration_validation test/test_registration_validation.cpp)
  target_include_directories(test_registration_validation PRIVATE include)
  target_link_libraries(test_registration_validation Eigen3::Eigen)

  ament_add_gtest(test_small_gicp_registration test/test_small_gicp_registration.cpp)
  target_include_directories(test_small_gicp_registration PRIVATE
    include
    ${small_gicp_SOURCE_DIR}/include
  )
  target_link_libraries(test_small_gicp_registration OpenMP::OpenMP_CXX Eigen3::Eigen)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
