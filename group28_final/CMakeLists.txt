cmake_minimum_required(VERSION 3.8)
project(group28_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf_tool REQUIRED)

# find_package(navigation_demo REQUIRED)

# include folder whish has all the header files
include_directories(include/group28_final)

# Add this line to include the config folder
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


#-----------------------------
# C++
#-----------------------------
add_executable(maze_runner src/maze_runner.cpp src/utils.cpp)

ament_target_dependencies(maze_runner ros2_aruco_interfaces 
mage_msgs sensor_msgs rclcpp rclcpp_action rclcpp_components nav2_msgs nav_msgs
 geometry_msgs ${FRAME_DEMO_INCLUDE_DEPENDS})

# #navigation_demo
# add_executable(navigation_demo src/navigation_demo.cpp)

# ament_target_dependencies(navigation_demo rclcpp rclcpp_action rclcpp_components nav2_msgs geometry_msgs)
# install(TARGETS navigation_demo DESTINATION lib/${PROJECT_NAME})


install(TARGETS maze_runner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
maze_runner
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  include
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
