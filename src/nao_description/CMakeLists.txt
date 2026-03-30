cmake_minimum_required(VERSION 3.8)
project(nao_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf rviz meshes config worlds texture
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/walk_point_to_point.py
    scripts/check_sim_health.py
    scripts/motion_player.py
    scripts/nao_vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Dossier motions/ (modules Python importables)
install(
  DIRECTORY scripts/motions
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()