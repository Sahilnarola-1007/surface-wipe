cmake_minimum_required(VERSION 3.8)
project(surface_wipe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wipe_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(wipe_node src/wipe_node.cpp)
target_include_directories(wipe_node PRIVATE include)
target_compile_features(wipe_node PUBLIC cxx_std_17)
ament_target_dependencies(wipe_node rclcpp wipe_msgs std_srvs geometry_msgs)

install(TARGETS wipe_node 
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wipe_trajectory test/test_wipe_trajectory.cpp)
  target_include_directories(test_wipe_trajectory PRIVATE include)
  target_compile_features(test_wipe_trajectory PUBLIC cxx_std_17)
endif()

ament_package()
