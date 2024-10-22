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

add_library(gramps_kd
  src/kinematics_server.cpp
)
add_dependencies(gramps_kd
    ${catkin_EXPORTED_TARGETS}
    gramps_kd_gencpp
)
target_link_libraries(gramps_kd
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
)
target_include_directories(gramps_kd SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(gramps_kd PUBLIC
  include
)

add_executable(kinematics_server
  src/kinematics_server_node.cpp
)
add_dependencies(gramps_kd
    ${catkin_EXPORTED_TARGETS}
    gramps_kd_gencpp
)
target_link_libraries(kinematics_server
  ${catkin_LIBRARIES}
  gramps_kd
  pinocchio::pinocchio
)
target_include_directories(kinematics_server SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(kinematics_server PUBLIC
  include
)
