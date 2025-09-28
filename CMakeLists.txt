cmake_minimum_required(VERSION 3.8)
project(lerobot_vr_controller)

# Set build type to Release with debug symbols
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 手动查找yaml-cpp，因为它可能不在ament依赖中
find_package(yaml-cpp REQUIRED)

# Create main VR controller executable
ament_auto_add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/vr_controller.cpp 
  src/kinematics.cpp
  src/trac_ik_kinematics.cpp
  src/joint_motor_convert.cpp
  src/vr_trigger_joint_convert.cpp
  src/robot_control_interface.cpp
  src/robot_communicate_interface.cpp
  src/joint_position_filter.cpp
  src/xlerobot_kinematics.cpp
)

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
  PROGRAMS 
    launch/display_robot.launch.py
    launch/vr_controller.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts
install(
  PROGRAMS
    script/reply_joint_state.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Create log test executable
  add_executable(log_test 
    src/log_test.cpp
  )
  
  # Include header files for log_test
  target_include_directories(log_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  install(TARGETS log_test
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_auto_package()
