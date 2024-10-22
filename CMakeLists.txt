cmake_minimum_required(VERSION 3.0.2)
project(gramps_kd)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  urdf
)

find_package(pinocchio REQUIRED)

add_service_files(
  FILES
  ForwardKinematics.srv
  InverseKinematics.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  message_runtime
  urdf
)

add_executable(kd_server
  src/kd_server_node.cpp
  src/kd_server.cpp
)
add_dependencies(kd_server
    ${catkin_EXPORTED_TARGETS}
    gramps_kd_gencpp
)
target_link_libraries(kd_server
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
)
target_include_directories(kd_server SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(kd_server PUBLIC
  include
)
