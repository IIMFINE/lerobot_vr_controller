cmake_minimum_required(VERSION 3.8)
project(lerobot_vr_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(yaml-cpp REQUIRED)

# Find all .cpp files in src directory
file(GLOB_RECURSE CPP_SOURCES "src/*.cpp")

# Add source files and create executable
ament_auto_add_executable(${PROJECT_NAME}_node ${CPP_SOURCES})

# Include header files
target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link yaml-cpp library
target_link_libraries(${PROJECT_NAME}_node ${YAML_CPP_LIBRARIES})
target_include_directories(${PROJECT_NAME}_node PRIVATE ${YAML_CPP_INCLUDE_DIRS})

# Install directories
install(
  DIRECTORY launch rviz model config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python launch files
install(
  PROGRAMS launch/display_robot.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
