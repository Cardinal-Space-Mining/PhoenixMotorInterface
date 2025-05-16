cmake_minimum_required(VERSION 3.8)
project(talon_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TalonCtrl.msg"
  "msg/TalonInfo.msg"
  DEPENDENCIES std_msgs
)

# Ensure message files are installed
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip lint check for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint if needed
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
