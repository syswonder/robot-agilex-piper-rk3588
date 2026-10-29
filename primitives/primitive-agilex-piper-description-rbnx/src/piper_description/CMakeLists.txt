cmake_minimum_required(VERSION 3.5)
project(piper_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Vendored subset of upstream piper_description. We only carry the
# directories actually needed for the rbnx grasp deploy:
#   * urdf/    — URDF + xacro, loaded by robot_state_publisher
#   * meshes/  — STL files referenced by `package://piper_description/meshes/...`
# The upstream package also ships launch/, rviz/, mujoco_model/, but
# rbnx owns its own launch (piper_description_rbnx/launch/ at the
# package root, used by scripts/start.sh), runs headless (no rviz),
# and doesn't simulate (no mujoco). Listing those here would make
# ament_cmake_symlink_install error out with "can't find DIRECTORY".
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
