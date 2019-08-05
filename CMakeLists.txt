cmake_minimum_required(VERSION 3.5)
project(rplidar_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_interfaces REQUIRED)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

include_directories(
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
  include
)

add_executable(rplidar_node src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
ament_target_dependencies(rplidar_node
  "rclcpp"
  "common_interfaces")

install(TARGETS rplidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch rviz sdk
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
