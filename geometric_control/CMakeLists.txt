cmake_minimum_required(VERSION 3.8)
project(geometric_control)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  include
#  ${Boost_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

set(dependencies
    rclcpp
    tf2
    tf2_ros
    std_msgs
    geometry_msgs
    nav_msgs
    trajectory_msgs
    px4_msgs
    controller_msgs)

add_library(geometric_control SHARED
  src/geometric_controller.cpp 
  src/nonlinear_attitude_control.cpp
  src/nonlinear_geometric_control.cpp
  src/jerk_tracking_control.cpp
)

ament_target_dependencies(geometric_control ${dependencies})

add_executable(geometric_controller_node src/geometric_controller_node.cpp)
ament_target_dependencies(geometric_controller_node ${dependencies})
target_link_libraries(geometric_controller_node geometric_control)


install(TARGETS geometric_controller_node DESTINATION lib/geometric_control)

install(
        TARGETS geometric_control
        EXPORT export_geometric_controller
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
