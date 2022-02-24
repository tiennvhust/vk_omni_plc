cmake_minimum_required(VERSION 3.0.2)
project(vk_omni_plc)

# Find catkin & any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  CppLinuxSerial
)

# Declare catkin package
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vk_omni_plc src/vk_omni_plc.cpp)

target_link_libraries(vk_omni_plc ${catkin_LIBRARIES})
