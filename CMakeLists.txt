cmake_minimum_required(VERSION 3.8)
project(orion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

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

ament_package()
