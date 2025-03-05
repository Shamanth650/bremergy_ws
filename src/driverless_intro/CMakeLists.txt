cmake_minimum_required(VERSION 3.8)
project(driverless_intro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone.msg"
  "msg/Track.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(cone_publisher src/cone_publisher.cpp)
ament_target_dependencies(cone_publisher 
  rclcpp 
  std_msgs 
  geometry_msgs 
  rosidl_default_runtime
)

add_executable(nearest_cone src/nearest_cone.cpp)
ament_target_dependencies(nearest_cone
  rclcpp
  std_msgs
  geometry_msgs
  rosidl_default_runtime 
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cone_publisher "${cpp_typesupport_target}")
target_link_libraries(nearest_cone "${cpp_typesupport_target}")

install(TARGETS
  cone_publisher
  nearest_cone
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  driverless_intro/cat_visualizer.py
  driverless_intro/cone_visualizer.py
  driverless_intro/cat_position.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY tracks
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
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
