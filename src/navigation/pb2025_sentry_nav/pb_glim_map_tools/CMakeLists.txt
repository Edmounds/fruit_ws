cmake_minimum_required(VERSION 3.8)
project(pb_glim_map_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)

add_executable(glim_ply_to_pcd src/glim_ply_to_pcd.cpp)
target_include_directories(glim_ply_to_pcd PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(glim_ply_to_pcd ${PCL_LIBRARIES})

add_library(map_conversion INTERFACE)
target_include_directories(map_conversion INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(map_conversion INTERFACE ${PCL_LIBRARIES})

install(TARGETS glim_ply_to_pcd DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_conversion test/test_map_conversion.cpp)
  target_link_libraries(test_map_conversion map_conversion)
endif()

ament_export_include_directories(include)
ament_package()
