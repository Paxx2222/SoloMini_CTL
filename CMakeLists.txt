cmake_minimum_required(VERSION 3.8)
project(solo_usb_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Curses REQUIRED)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIRS}
)

# SOLO driver library
add_library(solo_driver
  src/solo_serial.cpp
  src/solo_driver.cpp
)

target_link_libraries(solo_driver
  ${Boost_LIBRARIES}
)

# SOLO TUI library
add_library(solo_tui
  src/solo_tui.cpp
)

target_link_libraries(solo_tui
  solo_driver
  ${CURSES_LIBRARIES}
)

# solo_tui_direct executable (Stage 1)
add_executable(solo_tui_direct
  src/solo_tui_direct_main.cpp
)

target_link_libraries(solo_tui_direct
  solo_tui
  solo_driver
  ${Boost_LIBRARIES}
  ${CURSES_LIBRARIES}
)

# simple_test_tui executable - Simple communication test
add_executable(simple_test_tui
  src/simple_test_tui.cpp
)

target_link_libraries(simple_test_tui
  solo_driver
  ${Boost_LIBRARIES}
  ${CURSES_LIBRARIES}
)

# debug_protocol executable - Protocol debugging tool
add_executable(debug_protocol
  src/debug_protocol.cpp
)

target_link_libraries(debug_protocol
  solo_driver
  ${Boost_LIBRARIES}
)

# Install targets
install(TARGETS
  solo_driver
  solo_tui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  solo_tui_direct
  simple_test_tui
  debug_protocol
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rules
install(FILES
  rules/99-solo.rules
  DESTINATION share/${PROJECT_NAME}/rules
)

# Install documentation
install(DIRECTORY
  documents/
  DESTINATION share/${PROJECT_NAME}/documents
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

ament_package()
