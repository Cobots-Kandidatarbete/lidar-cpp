cmake_minimum_required(VERSION 3.8)
project(lidar_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(custom REQUIRED) 
find_package( OpenCV REQUIRED )

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar src/lidar_publisher.cpp)
ament_target_dependencies(lidar
  realsense2
  rclcpp
  sensor_msgs
  PCL
)

add_executable(lidar_server src/lidar_server.cpp)
ament_target_dependencies(lidar_server
  realsense2
  rclcpp
  sensor_msgs
  PCL
  custom
)

add_executable(lidar_client src/lidar_client.cpp)
ament_target_dependencies(lidar_client
  rclcpp
  sensor_msgs
  custom
)

add_executable(tmp src/tmp.cpp)
ament_target_dependencies(tmp
  rclcpp
  sensor_msgs
  realsense2
  PCL
  custom
)


target_link_libraries(lidar realsense2)
target_link_libraries(lidar_server realsense2 ${PCL_LIBRARIES})
target_link_libraries(tmp realsense2 ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS
  lidar
  lidar_server
  lidar_client
  tmp
  DESTINATION lib/${PROJECT_NAME})

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