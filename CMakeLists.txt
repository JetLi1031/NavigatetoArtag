cmake_minimum_required(VERSION 3.8)
project(robotnavigate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

                        
#add_executable(robotnav src/robotnavigater.cpp)

add_executable(batterypub src/battery_pub.cpp)
ament_target_dependencies(batterypub sensor_msgs rclcpp example_interfaces)

add_library(navigator src/robotnavigater.cpp)
ament_target_dependencies(navigator nav2_msgs rclcpp rclcpp_action tf2)

add_executable(docking src/dockmatch.cpp)
target_link_libraries(docking navigator)
ament_target_dependencies(docking geometry_msgs sensor_msgs rclcpp tf2_ros)

install(TARGETS
   batterypub
   docking
   DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY src scripts
  DESTINATION share/${PROJECT_NAME}
)



install(PROGRAMS
	scripts/ar2tf.py
	DESTINATION lib/${PROJECT_NAME}
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
