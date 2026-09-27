cmake_minimum_required(VERSION 3.5)
project(wheeltec_nav2)

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
add_compile_options(-g)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(v550_navigation REQUIRED)

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

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map rviz
  DESTINATION share/${PROJECT_NAME}
)

# Original microphone navigation uses this filename. Install current V550 tuning
# under that compatibility name without modifying any microphone driver files.
get_filename_component(V550_SHARE "${v550_navigation_DIR}" DIRECTORY)
install(FILES "${V550_SHARE}/config/nav2_v550.yaml"
  DESTINATION share/${PROJECT_NAME}/param/wheeltec_params
  RENAME param_mini_mec.yaml)
ament_package()
