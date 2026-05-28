cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)  
find_package(interactive_markers REQUIRED) 

# IK solver node
add_executable(ik_solver src/ik_solver.cpp)
target_include_directories(ik_solver PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ik_solver PUBLIC
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  interactive_markers::interactive_markers
)

install(TARGETS ik_solver
  DESTINATION lib/${PROJECT_NAME}
)

# Install URDF and launch files (only dirs that exist)
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
