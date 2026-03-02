cmake_minimum_required(VERSION 3.8)
project(orion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

foreach(dir config launch meshes urdf)
  install(
    DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir}
  )
endforeach()

# MoveItConfigsBuilder 据此推断 URDF/SRDF 路径，避免 WARNING
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pybullet_follow_joint_trajectory_server.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(orion_mtc_node src/orion_mtc_node.cpp)
ament_target_dependencies(orion_mtc_node
  moveit_task_constructor_core
  rclcpp
  moveit_ros_planning_interface
)
target_compile_features(orion_mtc_node PUBLIC cxx_std_17)

install(TARGETS orion_mtc_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
