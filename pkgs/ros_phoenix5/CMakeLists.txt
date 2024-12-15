cmake_minimum_required(VERSION 3.8)
project(robot_phoenix5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_types REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_executable(main src/main.cpp)
ament_target_dependencies(main rclcpp std_msgs custom_types)
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/Phoenix5-Linux-Example/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/libs/Phoenix5-Linux-Example/lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/libs/Phoenix5-Linux-Example/lib/arm64)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/libs/Phoenix5-Linux-Example/lib/x86-64)
endif()


 
target_link_libraries(main ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)


install(FILES 
  ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so
  ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so
  ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()

install(CODE "execute_process(COMMAND patchelf_cmake_build_step.sh \"${CMAKE_INSTALL_PREFIX}\" \"${CMAKE_INSTALL_PREFIX}/libCTRE_PhoenixTools.so\")")

