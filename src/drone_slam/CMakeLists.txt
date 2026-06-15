cmake_minimum_required(VERSION 3.8)
project(drone_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ── Instalar launch files y configuración ──────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
# ───────────────────────────────────────────────────────────────────────
# ── Instalar scripts Python como ejecutables ─────────────────────────────
# Esto permite llamarlos con `ros2 run drone_slam camera_info_pub`
install(PROGRAMS
  src/camera_info_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_info_pub
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
