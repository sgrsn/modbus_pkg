cmake_minimum_required(VERSION 3.0.2)
project(modbus_pkg)

## Compile as C++11, supported in ROS Noetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_modbus_pkg
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(modbus_node src/modbus_node.cpp)
target_link_libraries(modbus_node ${catkin_LIBRARIES} modbus)

install(TARGETS modbus_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)