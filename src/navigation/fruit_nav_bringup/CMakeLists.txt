cmake_minimum_required(VERSION 3.8)
project(fruit_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  rviz
)

