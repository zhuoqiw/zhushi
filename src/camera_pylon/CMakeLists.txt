cmake_minimum_required(VERSION 3.5)
project(camera_pylon)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pylon REQUIRED)
find_package(OpenCV REQUIRED)

add_library(camera_pylon SHARED src/camera_pylon.cpp)

target_include_directories(camera_pylon
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(camera_pylon
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} pylon::pylon)

rclcpp_components_register_node(camera_pylon
  PLUGIN "camera_pylon::CameraPylon"
  EXECUTABLE camera_pylon_node
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

install(TARGETS camera_pylon EXPORT export_camera_pylon DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_camera_pylon HAS_LIBRARY_TARGET)

ament_package()
