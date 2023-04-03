cmake_minimum_required(VERSION 3.8)
project(libpointmatcher_ros)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(libpointmatcher CONFIG)

include_directories(
    include
    ${libpointmatcher_INCLUDE_DIRS}
)

add_library(pointmatcher_ros
    src/PointMatcher_ROS.cpp
)

ament_target_dependencies(pointmatcher_ros
    rclcpp
    sensor_msgs
    nav_msgs
    tf2_eigen
)
target_link_libraries(pointmatcher_ros
    ${libpointmatcher_LIBRARIES}
)

install(DIRECTORY include
    DESTINATION include/${PROJECT_NAME}
)
install(TARGETS pointmatcher_ros
    EXPORT export_pointmatcher_ros
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

ament_export_include_directories("include/${PROJECT_NAME}/include")
ament_export_libraries(pointmatcher_ros)
ament_export_dependencies(rclcpp sensor_msgs nav_msgs tf2_eigen)

ament_package()
